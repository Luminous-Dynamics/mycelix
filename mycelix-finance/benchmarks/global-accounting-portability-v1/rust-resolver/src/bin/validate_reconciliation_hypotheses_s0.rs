use jsonschema::draft202012;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env,
    error::Error,
    fs,
    path::PathBuf,
};

fn main() {
    if let Err(error) = run() {
        eprintln!("reconciliation-hypotheses-s0: {error}");
        std::process::exit(2);
    }
}

fn read_bytes(path: PathBuf) -> Result<Vec<u8>, Box<dyn Error>> {
    Ok(fs::read(path)?)
}

fn read_json(path: PathBuf) -> Result<Value, Box<dyn Error>> {
    Ok(serde_json::from_slice(&read_bytes(path)?)?)
}

fn sha256(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

fn canonical_digest(value: &Value) -> Result<String, String> {
    let bytes = serde_json::to_vec(value)
        .map_err(|error| format!("could not serialize canonical JSON value: {error}"))?;
    Ok(sha256(&bytes))
}

fn validate_instance(label: &str, schema: &Value, instance: &Value) -> Result<(), String> {
    draft202012::meta::validate(schema)
        .map_err(|error| format!("{label}: invalid Draft 2020-12 schema: {error}"))?;
    let validator = draft202012::options()
        .should_validate_formats(true)
        .build(schema)
        .map_err(|error| format!("{label}: could not compile schema: {error}"))?;
    let errors = validator
        .iter_errors(instance)
        .map(|error| format!("{} at {}", error, error.instance_path()))
        .collect::<Vec<_>>();
    if errors.is_empty() {
        Ok(())
    } else {
        Err(format!("{label}: {}", errors.join("; ")))
    }
}

fn array<'a>(value: &'a Value, key: &str) -> Result<&'a [Value], String> {
    value
        .get(key)
        .and_then(Value::as_array)
        .map(Vec::as_slice)
        .ok_or_else(|| format!("missing or invalid array: {key}"))
}

fn string<'a>(value: &'a Value, key: &str) -> Result<&'a str, String> {
    value
        .get(key)
        .and_then(Value::as_str)
        .ok_or_else(|| format!("missing or invalid string: {key}"))
}

fn optional_string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn integer(value: &Value, key: &str) -> Result<i64, String> {
    value
        .get(key)
        .and_then(Value::as_i64)
        .ok_or_else(|| format!("missing or invalid integer: {key}"))
}

fn find_by_ref<'a>(
    values: &'a [Value],
    field: &str,
    reference: &str,
) -> Option<&'a Value> {
    values
        .iter()
        .find(|value| optional_string(value, field) == Some(reference))
}

fn source_item_by_ref<'a>(run: &'a Value, reference: &str) -> Result<&'a Value, String> {
    find_by_ref(array(run, "source_items")?, "source_item_ref", reference)
        .ok_or_else(|| format!("unknown source_item_ref {reference}"))
}

fn criterion_by_ref<'a>(
    rule_set: &'a Value,
    reference: &str,
) -> Result<&'a Value, String> {
    find_by_ref(array(rule_set, "criteria")?, "criterion_ref", reference)
        .ok_or_else(|| format!("unknown criterion_ref {reference}"))
}

fn criterion_value(item: &Value, kind: &str) -> Result<Option<String>, String> {
    let key = match kind {
        "amount" => return Ok(Some(integer(item, "amount_minor")?.to_string())),
        "currency" => "currency",
        "entity" => "entity_ref",
        "account" => "account_ref",
        "direction" => "direction",
        "date" => "transaction_date",
        "reference" => "external_reference",
        "counterparty" => "counterparty_ref",
        "transaction_type" => "transaction_type",
        other => return Err(format!("unsupported criterion kind {other}")),
    };
    Ok(item.get(key).and_then(Value::as_str).map(ToOwned::to_owned))
}

fn date_to_days(value: &str) -> Result<i64, String> {
    let mut parts = value.split('-');
    let mut year = parts.next().ok_or("missing year")?.parse::<i64>().map_err(|_| "invalid year")?;
    let month = parts.next().ok_or("missing month")?.parse::<i64>().map_err(|_| "invalid month")?;
    let day = parts.next().ok_or("missing day")?.parse::<i64>().map_err(|_| "invalid day")?;
    if parts.next().is_some() || !(1..=12).contains(&month) || !(1..=31).contains(&day) {
        return Err(format!("invalid ISO date {value}"));
    }
    year -= if month <= 2 { 1 } else { 0 };
    let era = year.div_euclid(400);
    let year_of_era = year - era * 400;
    let adjusted_month = month + if month > 2 { -3 } else { 9 };
    let day_of_year = (153 * adjusted_month + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    Ok(era * 146_097 + day_of_era)
}

fn criterion_outcome(
    kind: &str,
    left: &[Option<String>],
    right: &[Option<String>],
    amount_tolerance: i64,
    date_tolerance: i64,
) -> Result<&'static str, String> {
    if left.is_empty() || right.is_empty() {
        return Ok("unknown");
    }
    if left.iter().chain(right.iter()).any(Option::is_none) {
        return Ok("unknown");
    }
    match kind {
        "amount" => {
            let left_sum = left.iter().map(|value| {
                value.as_ref().ok_or("missing amount value")?.parse::<i64>()
                    .map_err(|_| "invalid amount value")
            }).collect::<Result<Vec<_>, _>>()?.into_iter().sum::<i64>();
            let right_sum = right.iter().map(|value| {
                value.as_ref().ok_or("missing amount value")?.parse::<i64>()
                    .map_err(|_| "invalid amount value")
            }).collect::<Result<Vec<_>, _>>()?.into_iter().sum::<i64>();
            Ok(if (left_sum - right_sum).abs() <= amount_tolerance { "match" } else { "mismatch" })
        }
        "date" => {
            let left_days = left.iter().map(|value| date_to_days(value.as_deref().ok_or("missing left date")?))
                .collect::<Result<Vec<_>, _>>()?;
            let right_days = right.iter().map(|value| date_to_days(value.as_deref().ok_or("missing right date")?))
                .collect::<Result<Vec<_>, _>>()?;
            let all_within = left_days.iter().all(|left_day| {
                right_days.iter().all(|right_day| (*left_day - *right_day).abs() <= date_tolerance)
            });
            Ok(if all_within { "match" } else { "mismatch" })
        }
        "currency" | "entity" | "account" | "direction" | "reference" | "counterparty" | "transaction_type" => {
            let values = left.iter().chain(right.iter()).filter_map(Option::as_ref).collect::<BTreeSet<_>>();
            Ok(if values.len() == 1 { "match" } else { "mismatch" })
        }
        other => Err(format!("unsupported criterion kind {other}")),
    }
}

fn expected_values_for_refs(
    run: &Value,
    refs: &[Value],
    kind: &str,
) -> Result<Vec<Option<String>>, String> {
    refs.iter()
        .map(|reference| {
            let reference = reference.as_str().ok_or("item reference must be a string")?;
            criterion_value(source_item_by_ref(run, reference)?, kind)
        })
        .collect()
}

fn json_optional_values(values: &[Option<String>]) -> Value {
    Value::Array(values.iter().map(|value| {
        value.as_ref().map(|value| Value::String(value.clone())).unwrap_or(Value::Null)
    }).collect())
}

fn refs_from_array(value: &Value, key: &str) -> Result<Vec<String>, String> {
    array(value, key)?
        .iter()
        .map(|item| item.as_str().map(ToOwned::to_owned).ok_or_else(|| format!("{key} must contain strings")))
        .collect()
}

fn set_from_array(value: &Value, key: &str) -> Result<BTreeSet<String>, String> {
    refs_from_array(value, key).map(|items| items.into_iter().collect())
}

fn cardinality_for(left_count: usize, right_count: usize) -> Option<&'static str> {
    match (left_count, right_count) {
        (1, 1) => Some("one_to_one"),
        (1, 2..) => Some("one_to_many"),
        (2.., 1) => Some("many_to_one"),
        (2.., 2..) => Some("many_to_many"),
        _ => None,
    }
}

fn validate_rule_set(run: &Value) -> Result<(), String> {
    let run_id = string(run, "run_id")?;
    let rule_set = run.get("rule_set").ok_or("missing rule_set")?;
    let criteria = array(rule_set, "criteria")?;
    let mut refs = BTreeSet::new();
    let mut total_weight = 0i64;
    for criterion in criteria {
        let reference = string(criterion, "criterion_ref")?;
        if !refs.insert(reference.to_owned()) {
            return Err(format!("{run_id}: duplicate criterion_ref {reference}"));
        }
        total_weight += integer(criterion, "weight_bps")?;
        if integer(criterion, "weight_bps")? < 0 {
            return Err(format!("{run_id}/{reference}: negative criterion weight"));
        }
    }
    if total_weight != 10_000 {
        return Err(format!("{run_id}: criterion weights sum to {total_weight}, expected 10000"));
    }
    if integer(rule_set, "review_threshold_bps")? > integer(rule_set, "auto_match_threshold_bps")? {
        return Err(format!("{run_id}: review threshold exceeds auto-match threshold"));
    }
    let mut cardinalities = BTreeSet::new();
    for value in array(rule_set, "allowed_cardinalities")? {
        let value = value.as_str().ok_or("allowed cardinality must be a string")?;
        if !cardinalities.insert(value.to_owned()) {
            return Err(format!("{run_id}: duplicate allowed cardinality {value}"));
        }
    }
    if cardinalities.is_empty() {
        return Err(format!("{run_id}: no allowed cardinalities"));
    }
    Ok(())
}

fn validate_input_run(run: &Value) -> Result<BTreeMap<String, &Value>, String> {
    let run_id = string(run, "run_id")?;
    validate_rule_set(run)?;
    let scope = run.get("scope").ok_or("missing run scope")?;
    let tenant = string(scope, "tenant_namespace")?;
    let entity = string(scope, "entity_ref")?;
    let account = string(scope, "account_ref")?;
    let knowledge_at = string(run, "knowledge_at")?;
    let period = run.get("period").ok_or("missing period")?;
    let period_start = string(period, "start")?;
    let period_end = string(period, "end")?;
    if period_start > period_end {
        return Err(format!("{run_id}: period start is after period end"));
    }

    let mut items = BTreeMap::new();
    let mut external_identities = BTreeSet::new();
    for item in array(run, "source_items")? {
        let item_ref = string(item, "source_item_ref")?;
        if items.insert(item_ref.to_owned(), item).is_some() {
            return Err(format!("{run_id}: duplicate source_item_ref {item_ref}"));
        }
        if string(item, "tenant_namespace")? != tenant
            || string(item, "entity_ref")? != entity
            || string(item, "account_ref")? != account
        {
            return Err(format!("{run_id}/{item_ref}: source item falls outside declared tenant/entity/account scope"));
        }
        if string(item, "received_at")? > knowledge_at {
            return Err(format!("{run_id}/{item_ref}: source item was received after the knowledge frontier"));
        }
        let transaction_date = string(item, "transaction_date")?;
        if transaction_date < period_start || transaction_date > period_end {
            return Err(format!("{run_id}/{item_ref}: transaction date is outside the run period"));
        }
        if integer(item, "amount_minor")? < 0 {
            return Err(format!("{run_id}/{item_ref}: negative source amount"));
        }
        let identity = (
            string(item, "source_system")?.to_owned(),
            string(item, "source_document_ref")?.to_owned(),
            string(item, "external_id")?.to_owned(),
            string(item, "source_side")?.to_owned(),
        );
        if !external_identities.insert(identity) {
            return Err(format!("{run_id}/{item_ref}: duplicate source identity"));
        }
        let reversal = optional_string(item, "reversal_of_external_id");
        if string(item, "transaction_type")? == "reversal" && reversal.is_none() {
            return Err(format!("{run_id}/{item_ref}: reversal has no reversal_of_external_id"));
        }
        if string(item, "transaction_type")? != "reversal" && reversal.is_some() {
            return Err(format!("{run_id}/{item_ref}: non-reversal carries reversal lineage"));
        }
    }
    let has_statement = items.values().any(|item| optional_string(item, "source_side") == Some("statement"));
    let has_ledger = items.values().any(|item| optional_string(item, "source_side") == Some("ledger"));
    if !has_statement || !has_ledger {
        return Err(format!("{run_id}: run must contain statement and ledger items"));
    }
    Ok(items)
}

fn pair_key(left: &str, right: &str) -> String {
    format!("{left}\u{1f}{right}")
}

fn expected_criterion_values(
    run: &Value,
    hypothesis: &Value,
    rule: &Value,
) -> Result<(Vec<Option<String>>, Vec<Option<String>>, &'static str), String> {
    let kind = string(rule, "kind")?;
    let left_refs = array(hypothesis, "left_item_refs")?;
    let right_refs = array(hypothesis, "right_item_refs")?;
    let left = expected_values_for_refs(run, left_refs, kind)?;
    let right = expected_values_for_refs(run, right_refs, kind)?;
    let rule_set = run.get("rule_set").ok_or("missing rule_set")?;
    let outcome = criterion_outcome(
        kind,
        &left,
        &right,
        integer(rule_set, "amount_tolerance_minor")?,
        integer(rule_set, "date_tolerance_days")?,
    )?;
    Ok((left, right, outcome))
}

fn validate_hypothesis(
    run: &Value,
    hypothesis: &Value,
    used_items: &mut BTreeSet<String>,
) -> Result<Value, String> {
    let run_id = string(run, "run_id")?;
    let hypothesis_ref = string(hypothesis, "hypothesis_ref")?;
    let left_refs = refs_from_array(hypothesis, "left_item_refs")?;
    let right_refs = refs_from_array(hypothesis, "right_item_refs")?;
    let expected_cardinality = cardinality_for(left_refs.len(), right_refs.len())
        .ok_or_else(|| format!("{run_id}/{hypothesis_ref}: unsupported group cardinality"))?;
    if string(hypothesis, "cardinality")? != expected_cardinality {
        return Err(format!("{run_id}/{hypothesis_ref}: declared cardinality does not match group sizes"));
    }
    let rule_set = run.get("rule_set").ok_or("missing rule set")?;
    if !array(rule_set, "allowed_cardinalities")?.iter()
        .any(|value| value.as_str() == Some(expected_cardinality))
    {
        return Err(format!("{run_id}/{hypothesis_ref}: cardinality is not allowed by the pinned rule set"));
    }

    let mut left_set = BTreeSet::new();
    let mut right_set = BTreeSet::new();
    for reference in &left_refs {
        if !left_set.insert(reference.clone()) {
            return Err(format!("{run_id}/{hypothesis_ref}: duplicate left item ref {reference}"));
        }
        if optional_string(source_item_by_ref(run, reference)?, "source_side") != Some("statement") {
            return Err(format!("{run_id}/{hypothesis_ref}: left item {reference} is not a statement item"));
        }
    }
    for reference in &right_refs {
        if !right_set.insert(reference.clone()) {
            return Err(format!("{run_id}/{hypothesis_ref}: duplicate right item ref {reference}"));
        }
        if optional_string(source_item_by_ref(run, reference)?, "source_side") != Some("ledger") {
            return Err(format!("{run_id}/{hypothesis_ref}: right item {reference} is not a ledger item"));
        }
    }

    let mut expected_pairs = BTreeSet::new();
    for left in &left_refs {
        for right in &right_refs {
            expected_pairs.insert(pair_key(left, right));
        }
    }
    let mut actual_pairs = BTreeSet::new();
    let mut pair_refs = BTreeSet::new();
    for pair in array(hypothesis, "candidate_pairs")? {
        let pair_ref = string(pair, "pair_ref")?;
        if !pair_refs.insert(pair_ref.to_owned()) {
            return Err(format!("{run_id}/{hypothesis_ref}: duplicate candidate pair ref {pair_ref}"));
        }
        let left = string(pair, "left_item_ref")?;
        let right = string(pair, "right_item_ref")?;
        if !actual_pairs.insert(pair_key(left, right)) {
            return Err(format!("{run_id}/{hypothesis_ref}: duplicate candidate pair"));
        }
    }
    if actual_pairs != expected_pairs {
        return Err(format!("{run_id}/{hypothesis_ref}: candidate pairs are not the complete left/right Cartesian product"));
    }

    let mut evals = BTreeMap::<String, &Value>::new();
    for evaluation in array(hypothesis, "criterion_evaluations")? {
        let reference = string(evaluation, "criterion_ref")?;
        if evals.insert(reference.to_owned(), evaluation).is_some() {
            return Err(format!("{run_id}/{hypothesis_ref}: duplicate criterion evaluation {reference}"));
        }
    }
    let rules = array(rule_set, "criteria")?;
    if evals.len() != rules.len() {
        return Err(format!("{run_id}/{hypothesis_ref}: criterion evaluation count differs from pinned rule set"));
    }

    let mut score = 0i64;
    let mut blocking_criterion = false;
    let mut review_criterion = false;
    let mut missing_or_required_failure = false;
    for rule in rules {
        let criterion_ref = string(rule, "criterion_ref")?;
        let evaluation = evals.get(criterion_ref)
            .ok_or_else(|| format!("{run_id}/{hypothesis_ref}: missing criterion evaluation {criterion_ref}"))?;
        let (left_values, right_values, outcome) = expected_criterion_values(run, hypothesis, rule)?;
        if evaluation["left_values"] != json_optional_values(&left_values)
            || evaluation["right_values"] != json_optional_values(&right_values)
        {
            return Err(format!("{run_id}/{hypothesis_ref}/{criterion_ref}: criterion evidence values do not match source items"));
        }
        if optional_string(evaluation, "outcome") != Some(outcome) {
            return Err(format!("{run_id}/{hypothesis_ref}/{criterion_ref}: reported outcome differs from deterministic outcome"));
        }
        match outcome {
            "match" => score += integer(rule, "weight_bps")?,
            "mismatch" if optional_string(rule, "mismatch_action") == Some("block") => blocking_criterion = true,
            "mismatch" if optional_string(rule, "mismatch_action") == Some("review") => review_criterion = true,
            "unknown" if optional_string(rule, "missing_action") == Some("block") => blocking_criterion = true,
            "unknown" if optional_string(rule, "missing_action") == Some("review") => review_criterion = true,
            _ => {}
        }
        if rule.get("required_for_auto_match").and_then(Value::as_bool) == Some(true)
            && outcome != "match"
        {
            missing_or_required_failure = true;
        }
    }
    if integer(hypothesis, "score_bps")? != score {
        return Err(format!("{run_id}/{hypothesis_ref}: reported score differs from criterion-weight sum"));
    }

    let below_review_threshold = score < integer(rule_set, "review_threshold_bps")?;
    let blocked_candidate = blocking_criterion || below_review_threshold;
    for pair in array(hypothesis, "candidate_pairs")? {
        let expected_state = if blocked_candidate { "rejected" } else { "candidate" };
        if optional_string(pair, "candidate_state") != Some(expected_state) {
            return Err(format!("{run_id}/{hypothesis_ref}: candidate pair state does not follow blocker/review-threshold evidence"));
        }
    }

    let mut allocated_left = BTreeMap::<String, i64>::new();
    let mut allocated_right = BTreeMap::<String, i64>::new();
    let mut allocation_refs = BTreeSet::new();
    let mut allocation_count = 0usize;
    for allocation in array(hypothesis, "allocations")? {
        allocation_count += 1;
        let allocation_ref = string(allocation, "allocation_ref")?;
        if !allocation_refs.insert(allocation_ref.to_owned()) {
            return Err(format!("{run_id}/{hypothesis_ref}: duplicate allocation ref {allocation_ref}"));
        }
        let left = string(allocation, "left_item_ref")?;
        let right = string(allocation, "right_item_ref")?;
        if !left_set.contains(left) || !right_set.contains(right) {
            return Err(format!("{run_id}/{hypothesis_ref}: allocation references an item outside the hypothesis group"));
        }
        if !expected_pairs.contains(&pair_key(left, right)) {
            return Err(format!("{run_id}/{hypothesis_ref}: allocation pair is not a candidate pair"));
        }
        let pair = array(hypothesis, "candidate_pairs")?.iter().find(|pair| {
            optional_string(pair, "left_item_ref") == Some(left)
                && optional_string(pair, "right_item_ref") == Some(right)
        }).ok_or("allocation pair not found")?;
        if optional_string(pair, "candidate_state") != Some("candidate") {
            return Err(format!("{run_id}/{hypothesis_ref}: allocation uses a rejected candidate pair"));
        }
        let amount = integer(allocation, "amount_minor")?;
        if amount <= 0 {
            return Err(format!("{run_id}/{hypothesis_ref}: allocation must be positive"));
        }
        *allocated_left.entry(left.to_owned()).or_default() += amount;
        *allocated_right.entry(right.to_owned()).or_default() += amount;
    }
    for (reference, total) in &allocated_left {
        let source_amount = integer(source_item_by_ref(run, reference)?, "amount_minor")?;
        if *total > source_amount {
            return Err(format!("{run_id}/{hypothesis_ref}: over-allocation of statement item {reference}"));
        }
    }
    for (reference, total) in &allocated_right {
        let source_amount = integer(source_item_by_ref(run, reference)?, "amount_minor")?;
        if *total > source_amount {
            return Err(format!("{run_id}/{hypothesis_ref}: over-allocation of ledger item {reference}"));
        }
    }

    let mut is_fully_allocated = true;
    for reference in &left_refs {
        let source_amount = integer(source_item_by_ref(run, reference)?, "amount_minor")?;
        if allocated_left.get(reference).copied().unwrap_or(0) != source_amount {
            is_fully_allocated = false;
        }
    }
    for reference in &right_refs {
        let source_amount = integer(source_item_by_ref(run, reference)?, "amount_minor")?;
        if allocated_right.get(reference).copied().unwrap_or(0) != source_amount {
            is_fully_allocated = false;
        }
    }

    let decision = string(hypothesis, "decision_state")?;
    if blocked_candidate {
        if decision != "blocked" || allocation_count != 0 {
            return Err(format!("{run_id}/{hypothesis_ref}: blocking criterion requires blocked state and zero allocations"));
        }
    } else if decision == "blocked" {
        return Err(format!("{run_id}/{hypothesis_ref}: blocked state has neither a blocking criterion nor a below-review-threshold score"));
    }

    if decision == "eligible_for_auto_match" {
        let mut positive_source_amounts = true;
        let mut reversal_sources = Vec::new();
        let mut contains_reversal = false;
        for reference in left_refs.iter().chain(right_refs.iter()) {
            let source = source_item_by_ref(run, reference)?;
            if integer(source, "amount_minor")? <= 0 {
                positive_source_amounts = false;
            }
            if optional_string(source, "transaction_type") == Some("reversal") {
                contains_reversal = true;
            }
            reversal_sources.push(source);
        }
        if blocking_criterion || review_criterion || missing_or_required_failure || score < integer(rule_set, "auto_match_threshold_bps")? || !is_fully_allocated || !positive_source_amounts {
            return Err(format!("{run_id}/{hypothesis_ref}: auto-match eligibility requirements are not satisfied"));
        }
        if contains_reversal {
            if reversal_sources.iter().any(|item| optional_string(item, "transaction_type") != Some("reversal")) {
                return Err(format!("{run_id}/{hypothesis_ref}: reversal cannot match a non-reversal item"));
            }
            let reversal_ids = reversal_sources.iter()
                .map(|item| optional_string(item, "reversal_of_external_id"))
                .collect::<BTreeSet<_>>();
            if reversal_ids.len() != 1 || reversal_ids.contains(&None) {
                return Err(format!("{run_id}/{hypothesis_ref}: reversal lineage differs across matched items"));
            }
        }
    } else if decision == "review_required" {
        let partial_allocation = allocation_count > 0 && !is_fully_allocated;
        if blocked_candidate
            || (!review_criterion && !missing_or_required_failure && !partial_allocation
                && score < integer(rule_set, "review_threshold_bps")?)
            || (!review_criterion && !missing_or_required_failure && !partial_allocation
                && score >= integer(rule_set, "auto_match_threshold_bps")? && is_fully_allocated)
        {
            return Err(format!("{run_id}/{hypothesis_ref}: review_required state is not justified by evidence, score or allocation residual"));
        }
    } else if decision == "ambiguous" {
        if blocked_candidate {
            return Err(format!("{run_id}/{hypothesis_ref}: blocked/low-score candidate cannot be labelled ambiguous"));
        }
        if allocation_count != 0 {
            return Err(format!("{run_id}/{hypothesis_ref}: ambiguous hypothesis must not allocate source items"));
        }
    } else if decision != "blocked" {
        return Err(format!("{run_id}/{hypothesis_ref}: unsupported decision state {decision}"));
    }

    if allocation_count > 0 && decision != "blocked" && decision != "ambiguous" {
        for reference in left_refs.iter().chain(right_refs.iter()) {
            if !used_items.insert(reference.clone()) {
                return Err(format!("{run_id}/{hypothesis_ref}: source item {reference} is reused by independent allocated hypotheses"));
            }
        }
    }

    let alternatives = set_from_array(hypothesis, "alternative_hypothesis_refs")?;
    if alternatives.contains(hypothesis_ref) {
        return Err(format!("{run_id}/{hypothesis_ref}: hypothesis cannot be its own alternative"));
    }

    Ok(json!({
        "hypothesis_count": 1,
        "eligible_for_auto_match_count": if decision == "eligible_for_auto_match" { 1 } else { 0 },
        "review_required_count": if decision == "review_required" { 1 } else { 0 },
        "ambiguous_count": if decision == "ambiguous" { 1 } else { 0 },
        "blocked_count": if decision == "blocked" { 1 } else { 0 },
        "candidate_pair_count": array(hypothesis, "candidate_pairs")?.len(),
        "allocation_count": allocation_count,
        "is_fully_allocated": is_fully_allocated,
        "score_bps": score
    }))
}

fn validate_alternative_sets(run: &Value, hypotheses: &[Value]) -> Result<(), String> {
    let run_id = string(run, "run_id")?;
    let rule_set = run.get("rule_set").ok_or("missing rule set")?;
    let margin = integer(rule_set, "ambiguity_margin_bps")?;
    let mut by_ref = BTreeMap::<String, &Value>::new();
    for hypothesis in hypotheses {
        let reference = string(hypothesis, "hypothesis_ref")?;
        if by_ref.insert(reference.to_owned(), hypothesis).is_some() {
            return Err(format!("{run_id}: duplicate hypothesis ref {reference}"));
        }
    }

    for index in 0..hypotheses.len() {
        for other_index in (index + 1)..hypotheses.len() {
            let left = &hypotheses[index];
            let right = &hypotheses[other_index];
            let left_refs = set_from_array(left, "left_item_refs")?
                .into_iter().chain(set_from_array(left, "right_item_refs")?).collect::<BTreeSet<_>>();
            let right_refs = set_from_array(right, "left_item_refs")?
                .into_iter().chain(set_from_array(right, "right_item_refs")?).collect::<BTreeSet<_>>();
            let overlaps = !left_refs.is_disjoint(&right_refs);
            let score_delta = (integer(left, "score_bps")? - integer(right, "score_bps")?).abs();
            let review_floor = integer(rule_set, "review_threshold_bps")?;
            let states_reviewable = optional_string(left, "decision_state") != Some("blocked")
                && optional_string(right, "decision_state") != Some("blocked")
                && integer(left, "score_bps")? >= review_floor
                && integer(right, "score_bps")? >= review_floor;
            let near_tie = overlaps && states_reviewable && score_delta <= margin;
            let left_ref = string(left, "hypothesis_ref")?;
            let right_ref = string(right, "hypothesis_ref")?;
            if near_tie {
                if optional_string(left, "decision_state") != Some("ambiguous")
                    || optional_string(right, "decision_state") != Some("ambiguous")
                {
                    return Err(format!("{run_id}: overlapping near-tie hypotheses must both remain ambiguous"));
                }
                if !set_from_array(left, "alternative_hypothesis_refs")?.contains(right_ref)
                    || !set_from_array(right, "alternative_hypothesis_refs")?.contains(left_ref)
                {
                    return Err(format!("{run_id}: ambiguous near-tie alternatives must reference each other"));
                }
            }
        }
    }

    for hypothesis in hypotheses {
        let reference = string(hypothesis, "hypothesis_ref")?;
        let alternatives = set_from_array(hypothesis, "alternative_hypothesis_refs")?;
        if optional_string(hypothesis, "decision_state") == Some("ambiguous") && alternatives.is_empty() {
            return Err(format!("{run_id}/{reference}: ambiguous hypothesis has no alternatives"));
        }
        for alternative_ref in alternatives {
            let alternative = by_ref.get(&alternative_ref)
                .ok_or_else(|| format!("{run_id}/{reference}: unknown alternative hypothesis {alternative_ref}"))?;
            if optional_string(alternative, "decision_state") != Some("ambiguous")
                || !set_from_array(alternative, "alternative_hypothesis_refs")?.contains(reference)
            {
                return Err(format!("{run_id}/{reference}: alternative link is not symmetric and ambiguous"));
            }
            let current_refs = set_from_array(hypothesis, "left_item_refs")?
                .into_iter().chain(set_from_array(hypothesis, "right_item_refs")?).collect::<BTreeSet<_>>();
            let alternative_refs = set_from_array(alternative, "left_item_refs")?
                .into_iter().chain(set_from_array(alternative, "right_item_refs")?).collect::<BTreeSet<_>>();
            let overlap = !current_refs.is_disjoint(&alternative_refs);
            let score_delta = (integer(hypothesis, "score_bps")? - integer(alternative, "score_bps")?).abs();
            if !overlap || score_delta > margin {
                return Err(format!("{run_id}/{reference}: alternative is not an overlapping near-tie candidate"));
            }
        }
    }
    Ok(())
}

fn validate_unmatched_items(
    run: &Value,
    result: &Value,
    hypotheses: &[Value],
) -> Result<(usize, i64), String> {
    let run_id = string(run, "run_id")?;
    let mut allocated = BTreeMap::<String, i64>::new();
    let mut associated = BTreeMap::<String, BTreeSet<String>>::new();
    let mut ambiguous_items = BTreeSet::new();
    let mut blocked_items = BTreeSet::new();
    let mut review_items = BTreeSet::new();
    for hypothesis in hypotheses {
        let hypothesis_ref = string(hypothesis, "hypothesis_ref")?.to_owned();
        let state = string(hypothesis, "decision_state")?;
        for reference in refs_from_array(hypothesis, "left_item_refs")?
            .into_iter()
            .chain(refs_from_array(hypothesis, "right_item_refs")?)
        {
            associated.entry(reference.clone()).or_default().insert(hypothesis_ref.clone());
            match state {
                "ambiguous" => { ambiguous_items.insert(reference); }
                "blocked" => { blocked_items.insert(reference); }
                "review_required" => { review_items.insert(reference); }
                _ => {}
            }
        }
        for allocation in array(hypothesis, "allocations")? {
            let amount = integer(allocation, "amount_minor")?;
            *allocated.entry(string(allocation, "left_item_ref")?.to_owned()).or_default() += amount;
            *allocated.entry(string(allocation, "right_item_ref")?.to_owned()).or_default() += amount;
        }
    }

    let mut unmatched = BTreeMap::<String, &Value>::new();
    let mut unresolved_total = 0i64;
    for item in array(result, "unmatched_items")? {
        let reference = string(item, "source_item_ref")?;
        if unmatched.insert(reference.to_owned(), item).is_some() {
            return Err(format!("{run_id}: duplicate unmatched item {reference}"));
        }
        let source = source_item_by_ref(run, reference)?;
        let expected_remaining = integer(source, "amount_minor")?
            - allocated.get(reference).copied().unwrap_or(0);
        if expected_remaining < 0 || integer(item, "unallocated_amount_minor")? != expected_remaining {
            return Err(format!("{run_id}/{reference}: unmatched residual differs from source amount minus allocations"));
        }
        let expected_hypotheses = associated.get(reference).cloned().unwrap_or_default();
        if set_from_array(item, "candidate_hypothesis_refs")? != expected_hypotheses {
            return Err(format!("{run_id}/{reference}: unmatched item candidate refs are incomplete or incorrect"));
        }

        let expected_disposition = if ambiguous_items.contains(reference) {
            "ambiguous"
        } else if blocked_items.contains(reference) {
            "blocked"
        } else if review_items.contains(reference) && expected_remaining > 0 {
            "residual_review"
        } else if expected_remaining > 0 {
            "unmatched"
        } else {
            return Err(format!("{run_id}/{reference}: fully allocated item is incorrectly listed as unmatched"));
        };
        if optional_string(item, "disposition") != Some(expected_disposition) {
            return Err(format!("{run_id}/{reference}: unmatched disposition does not follow hypothesis state"));
        }
        unresolved_total += expected_remaining;
    }

    for item in array(run, "source_items")? {
        let reference = string(item, "source_item_ref")?;
        let remaining = integer(item, "amount_minor")?
            - allocated.get(reference).copied().unwrap_or(0);
        if remaining > 0 && !unmatched.contains_key(reference) {
            return Err(format!("{run_id}/{reference}: residual source amount was silently dropped"));
        }
        if remaining == 0 && unmatched.contains_key(reference) {
            return Err(format!("{run_id}/{reference}: fully allocated source item is listed as unmatched"));
        }
    }

    Ok((unmatched.len(), unresolved_total))
}

fn validate_run_result(
    run: &Value,
    result: &Value,
) -> Result<Value, String> {
    let run_id = string(run, "run_id")?;
    if string(result, "run_id")? != run_id {
        return Err(format!("{run_id}: input/output run IDs differ"));
    }
    let rule_digest = canonical_digest(run.get("rule_set").ok_or("missing input rule set")?)?;
    if string(result, "rule_set_sha256")? != rule_digest {
        return Err(format!("{run_id}: rule-set SHA-256 mismatch"));
    }

    let hypotheses = array(result, "hypotheses")?;
    let mut hypothesis_ids = BTreeSet::new();
    let mut used_items = BTreeSet::new();
    let mut counters = BTreeMap::<String, usize>::new();
    for hypothesis in hypotheses {
        let reference = string(hypothesis, "hypothesis_ref")?;
        if !hypothesis_ids.insert(reference.to_owned()) {
            return Err(format!("{run_id}: duplicate hypothesis {reference}"));
        }
        let stats = validate_hypothesis(run, hypothesis, &mut used_items)?;
        for key in [
            "hypothesis_count",
            "eligible_for_auto_match_count",
            "review_required_count",
            "ambiguous_count",
            "blocked_count",
            "candidate_pair_count",
            "allocation_count",
        ] {
            *counters.entry(key.to_owned()).or_default() += stats[key].as_u64().unwrap_or(0) as usize;
        }
    }
    validate_alternative_sets(run, hypotheses)?;
    let (unmatched_count, unresolved_amount) = validate_unmatched_items(run, result, hypotheses)?;
    let reused_item_count = 0usize;
    let mut findings = BTreeSet::new();
    if counters.get("ambiguous_count").copied().unwrap_or(0) > 0 {
        findings.insert("ambiguous_alternatives_retained");
    }
    if array(result, "unmatched_items")?.iter().any(|item| optional_string(item, "disposition") == Some("residual_review")) {
        findings.insert("partial_allocation_requires_review");
    }
    if counters.get("blocked_count").copied().unwrap_or(0) > 0 {
        findings.insert("blocked_candidate_preserved");
    }
    if unmatched_count > 0 {
        findings.insert("unmatched_source_item_preserved");
    }

    Ok(json!({
        "run_id": run_id,
        "hypothesis_count": counters.get("hypothesis_count").copied().unwrap_or(0),
        "eligible_for_auto_match_count": counters.get("eligible_for_auto_match_count").copied().unwrap_or(0),
        "review_required_count": counters.get("review_required_count").copied().unwrap_or(0),
        "ambiguous_count": counters.get("ambiguous_count").copied().unwrap_or(0),
        "blocked_count": counters.get("blocked_count").copied().unwrap_or(0),
        "candidate_pair_count": counters.get("candidate_pair_count").copied().unwrap_or(0),
        "allocation_count": counters.get("allocation_count").copied().unwrap_or(0),
        "unmatched_item_count": unmatched_count,
        "unresolved_amount_minor": unresolved_amount,
        "reused_source_item_count": reused_item_count,
        "finding_codes": findings.into_iter().collect::<Vec<_>>()
    }))
}

fn build_report(
    input: &Value,
    output: &Value,
    input_digest: String,
    output_digest: String,
) -> Result<Value, String> {
    if string(output, "source_bundle_sha256")? != input_digest {
        return Err("output source_bundle_sha256 does not match exact input bundle bytes".to_owned());
    }
    let matcher = output.get("matcher_binding").ok_or("missing matcher binding")?;
    match string(matcher, "engine_state")? {
        "not_executed_synthetic_fixture" => {
            if !matcher.get("engine_sha256").is_none_or(Value::is_null)
                || !matcher.get("engine_artifact_ref").is_none_or(Value::is_null)
            {
                return Err("unexecuted synthetic matcher fixture must not claim an engine digest or artifact".to_owned());
            }
        }
        "source_digest_verified" | "binary_digest_verified" => {
            if optional_string(matcher, "engine_sha256").is_none()
                || optional_string(matcher, "engine_artifact_ref").is_none()
            {
                return Err("verified matcher binding requires a digest and artifact reference".to_owned());
            }
        }
        other => return Err(format!("unsupported matcher engine_state {other}")),
    }
    let input_runs = array(input, "runs")?;
    let output_runs = array(output, "run_results")?;
    if input_runs.len() != output_runs.len() {
        return Err(format!("input/output run counts differ: {} vs {}", input_runs.len(), output_runs.len()));
    }
    let mut input_ids = BTreeSet::new();
    let mut output_ids = BTreeSet::new();
    let mut run_results = Vec::new();
    let total_keys = [
        "hypothesis_count",
        "eligible_for_auto_match_count",
        "review_required_count",
        "ambiguous_count",
        "blocked_count",
        "candidate_pair_count",
        "allocation_count",
        "unmatched_item_count",
        "reused_source_item_count",
        "unresolved_amount_minor",
    ];
    let mut totals = BTreeMap::<String, i64>::new();

    for run in input_runs {
        let run_id = string(run, "run_id")?;
        if !input_ids.insert(run_id.to_owned()) {
            return Err(format!("duplicate input run {run_id}"));
        }
        validate_input_run(run)?;
        let result = output_runs.iter()
            .find(|item| optional_string(item, "run_id") == Some(run_id))
            .ok_or_else(|| format!("missing output run {run_id}"))?;
        output_ids.insert(string(result, "run_id")?.to_owned());
        let run_report = validate_run_result(run, result)?;
        for key in total_keys {
            *totals.entry(key.to_owned()).or_default() += run_report[key].as_i64().unwrap_or(0);
        }
        run_results.push(run_report);
    }
    if input_ids != output_ids {
        return Err("input and output run ID sets differ".to_owned());
    }
    let total_value = total_keys.iter().map(|key| {
        ((*key).to_owned(), Value::from(totals.get(*key).copied().unwrap_or(0)))
    }).collect::<serde_json::Map<_, _>>();

    Ok(json!({
        "report_id": "MYCELIX-RECON-HYPOTHESES-S0-VALIDATION",
        "schema_version": "1.0.0",
        "claim_ceiling": "deterministic_hypothesis_contract_validation_not_match_quality_or_ledger_posting",
        "source_bundle_sha256": input_digest,
        "hypotheses_bundle_sha256": output_digest,
        "run_count": run_results.len(),
        "totals": total_value,
        "run_results": run_results
    }))
}

fn run() -> Result<(), Box<dyn Error>> {
    let default_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../reconciliation-hypotheses-v1");
    let mut data_dir = default_dir;
    let mut output_path: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--data-dir" => data_dir = PathBuf::from(args.next().ok_or("--data-dir requires a directory")?),
            "--output" => output_path = Some(PathBuf::from(args.next().ok_or("--output requires a file path")?)),
            "--help" | "-h" => {
                println!(
                    "Validates reconciliation hypotheses without confirming matches or posting ledger entries.\n\n\
                     --data-dir PATH  Reconciliation package directory (default: sibling reconciliation-hypotheses-v1)\n\
                     --output PATH    Write deterministic report JSON to PATH\n\
                     --help           Show usage"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}; use --help").into()),
        }
    }

    let input_schema: Value = read_json(data_dir.join("reconciliation_input_v1.0.0.schema.json"))?;
    let output_schema: Value = read_json(data_dir.join("reconciliation_hypotheses_v1.0.0.schema.json"))?;
    let report_schema: Value = read_json(data_dir.join("reconciliation_validation_report_v1.0.0.schema.json"))?;
    let input_path = data_dir.join("fixtures/reconciliation_inputs_s0_v1.0.0.json");
    let output_input_path = data_dir.join("fixtures/reconciliation_hypotheses_s0_v1.0.0.json");
    let input_bytes = read_bytes(input_path)?;
    let output_bytes = read_bytes(output_input_path)?;
    let input: Value = serde_json::from_slice(&input_bytes)?;
    let output: Value = serde_json::from_slice(&output_bytes)?;
    validate_instance("reconciliation input bundle", &input_schema, &input)?;
    validate_instance("reconciliation hypotheses", &output_schema, &output)?;
    let input_digest = sha256(&input_bytes);
    let output_digest = sha256(&output_bytes);
    let report = build_report(&input, &output, input_digest, output_digest)?;
    validate_instance("reconciliation validation report", &report_schema, &report)?;
    let serialized = serde_json::to_vec_pretty(&report)?;
    match output_path {
        Some(path) => {
            fs::write(path, &serialized)?;
            println!("Wrote reconciliation validation report for {} runs.", report["run_count"]);
        }
        None => println!("{}", String::from_utf8(serialized)?),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (Value, Value, Value, Value) {
        let data_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../reconciliation-hypotheses-v1");
        (
            read_json(data_dir.join("fixtures/reconciliation_inputs_s0_v1.0.0.json")).unwrap(),
            read_json(data_dir.join("fixtures/reconciliation_hypotheses_s0_v1.0.0.json")).unwrap(),
            read_json(data_dir.join("reconciliation_input_v1.0.0.schema.json")).unwrap(),
            read_json(data_dir.join("reconciliation_hypotheses_v1.0.0.schema.json")).unwrap(),
        )
    }

    fn run_result<'a>(report: &'a Value, run_id: &str) -> &'a Value {
        array(report, "run_results").unwrap().iter()
            .find(|result| optional_string(result, "run_id") == Some(run_id)).unwrap()
    }

    fn build(input: &Value, output: &Value) -> Value {
        build_report(input, output, output["source_bundle_sha256"].as_str().unwrap().to_owned(), "f".repeat(64)).unwrap()
    }

    #[test]
    fn all_eight_reconciliation_runs_validate() {
        let (input, output, input_schema, output_schema) = fixtures();
        validate_instance("input", &input_schema, &input).unwrap();
        validate_instance("output", &output_schema, &output).unwrap();
        let report = build(&input, &output);
        assert_eq!(report["run_count"], 8);
        assert_eq!(report["totals"]["hypothesis_count"], 9);
        assert_eq!(report["totals"]["eligible_for_auto_match_count"], 5);
        assert_eq!(report["totals"]["review_required_count"], 1);
        assert_eq!(report["totals"]["ambiguous_count"], 2);
        assert_eq!(report["totals"]["blocked_count"], 1);
    }

    #[test]
    fn one_to_many_allocation_conserves_both_sides() {
        let (input, output, _, _) = fixtures();
        let report = build(&input, &output);
        let run = run_result(&report, "RECON-002-ONE-TO-MANY");
        assert_eq!(run["allocation_count"], 2);
        assert_eq!(run["eligible_for_auto_match_count"], 1);
        assert_eq!(run["unmatched_item_count"], 0);
    }

    #[test]
    fn many_to_one_allocation_conserves_both_sides() {
        let (input, output, _, _) = fixtures();
        let report = build(&input, &output);
        let run = run_result(&report, "RECON-003-MANY-TO-ONE");
        assert_eq!(run["allocation_count"], 2);
        assert_eq!(run["eligible_for_auto_match_count"], 1);
    }

    #[test]
    fn many_to_many_allocation_matrix_conserves_all_items() {
        let (input, output, _, _) = fixtures();
        let report = build(&input, &output);
        let run = run_result(&report, "RECON-004-MANY-TO-MANY");
        assert_eq!(run["allocation_count"], 3);
        assert_eq!(run["eligible_for_auto_match_count"], 1);
    }

    #[test]
    fn partial_settlement_retains_unallocated_residual_for_review() {
        let (input, output, _, _) = fixtures();
        let report = build(&input, &output);
        let run = run_result(&report, "RECON-005-PARTIAL-SETTLEMENT");
        assert_eq!(run["review_required_count"], 1);
        assert_eq!(run["unresolved_amount_minor"], 3000);
        assert_eq!(run["unmatched_item_count"], 1);
    }

    #[test]
    fn equal_score_alternatives_remain_ambiguous_and_unallocated() {
        let (input, output, _, _) = fixtures();
        let report = build(&input, &output);
        let run = run_result(&report, "RECON-006-AMBIGUOUS-ALTERNATIVES");
        assert_eq!(run["ambiguous_count"], 2);
        assert_eq!(run["allocation_count"], 0);
        assert_eq!(run["unmatched_item_count"], 3);
    }

    #[test]
    fn currency_conflict_is_blocked_without_conversion_evidence() {
        let (input, output, _, _) = fixtures();
        let report = build(&input, &output);
        let run = run_result(&report, "RECON-007-CURRENCY-CONFLICT");
        assert_eq!(run["blocked_count"], 1);
        assert_eq!(run["allocation_count"], 0);
        assert_eq!(run["unresolved_amount_minor"], 20000);
    }

    #[test]
    fn reversal_match_requires_equal_reversal_lineage() {
        let (input, output, _, _) = fixtures();
        let report = build(&input, &output);
        let run = run_result(&report, "RECON-008-REVERSAL");
        assert_eq!(run["eligible_for_auto_match_count"], 1);
    }

    #[test]
    fn over_allocation_is_rejected() {
        let (input, mut output, _, _) = fixtures();
        let target = output["run_results"].as_array_mut().unwrap().iter_mut()
            .find(|run| optional_string(run, "run_id") == Some("RECON-001-ONE-TO-ONE")).unwrap();
        target["hypotheses"][0]["allocations"][0]["amount_minor"] = Value::from(250001);
        assert!(build_report(&input, &output, output["source_bundle_sha256"].as_str().unwrap().to_owned(), "f".repeat(64)).is_err());
    }

    #[test]
    fn selected_source_item_cannot_be_reused_by_independent_hypotheses() {
        let (input, mut output, _, _) = fixtures();
        let target = output["run_results"].as_array_mut().unwrap().iter_mut()
            .find(|run| optional_string(run, "run_id") == Some("RECON-001-ONE-TO-ONE")).unwrap();
        let mut duplicate = target["hypotheses"][0].clone();
        duplicate["hypothesis_ref"] = Value::String("HYP-001-DUPLICATE".to_owned());
        duplicate["candidate_pairs"][0]["pair_ref"] = Value::String("PAIR-001-DUPLICATE".to_owned());
        duplicate["allocations"][0]["allocation_ref"] = Value::String("ALLOC-001-DUPLICATE".to_owned());
        target["hypotheses"].as_array_mut().unwrap().push(duplicate);
        assert!(build_report(&input, &output, output["source_bundle_sha256"].as_str().unwrap().to_owned(), "f".repeat(64)).is_err());
    }

    #[test]
    fn ambiguity_cannot_be_resolved_by_arbitrary_allocation() {
        let (input, mut output, _, _) = fixtures();
        let target = output["run_results"].as_array_mut().unwrap().iter_mut()
            .find(|run| optional_string(run, "run_id") == Some("RECON-006-AMBIGUOUS-ALTERNATIVES")).unwrap();
        target["hypotheses"][0]["allocations"] = json!([{
            "allocation_ref":"ALLOC-AMBIG",
            "left_item_ref":"R6-S",
            "right_item_ref":"R6-L1",
            "amount_minor":10000
        }]);
        assert!(build_report(&input, &output, output["source_bundle_sha256"].as_str().unwrap().to_owned(), "f".repeat(64)).is_err());
    }

    #[test]
    fn missing_alternative_link_is_rejected_for_ambiguous_hypothesis() {
        let (input, mut output, _, _) = fixtures();
        let target = output["run_results"].as_array_mut().unwrap().iter_mut()
            .find(|run| optional_string(run, "run_id") == Some("RECON-006-AMBIGUOUS-ALTERNATIVES")).unwrap();
        target["hypotheses"][0]["alternative_hypothesis_refs"] = json!([]);
        assert!(build_report(&input, &output, output["source_bundle_sha256"].as_str().unwrap().to_owned(), "f".repeat(64)).is_err());
    }

    #[test]
    fn source_currency_mismatch_blocks_even_when_amounts_match() {
        let (input, mut output, _, _) = fixtures();
        let target = output["run_results"].as_array_mut().unwrap().iter_mut()
            .find(|run| optional_string(run, "run_id") == Some("RECON-007-CURRENCY-CONFLICT")).unwrap();
        target["hypotheses"][0]["criterion_evaluations"].as_array_mut().unwrap()
            .iter_mut().find(|evaluation| optional_string(evaluation, "criterion_ref") == Some("CR-CURRENCY")).unwrap()["outcome"] = Value::String("match".to_owned());
        assert!(build_report(&input, &output, output["source_bundle_sha256"].as_str().unwrap().to_owned(), "f".repeat(64)).is_err());
    }

    #[test]
    fn rule_weights_must_sum_to_ten_thousand_basis_points() {
        let (mut input, output, _, _) = fixtures();
        input["runs"][0]["rule_set"]["criteria"][0]["weight_bps"] = Value::from(2499);
        assert!(build_report(&input, &output, output["source_bundle_sha256"].as_str().unwrap().to_owned(), "f".repeat(64)).is_err());
    }

    #[test]
    fn output_must_bind_exact_input_bytes() {
        let (input, mut output, _, _) = fixtures();
        output["source_bundle_sha256"] = Value::String("0".repeat(64));
        assert!(build_report(&input, &output, "1".repeat(64), "f".repeat(64)).is_err());
    }

    #[test]
    fn unmatched_residual_cannot_be_silently_omitted() {
        let (input, mut output, _, _) = fixtures();
        let target = output["run_results"].as_array_mut().unwrap().iter_mut()
            .find(|run| optional_string(run, "run_id") == Some("RECON-005-PARTIAL-SETTLEMENT")).unwrap();
        target["unmatched_items"] = json!([]);
        assert!(build_report(&input, &output, output["source_bundle_sha256"].as_str().unwrap().to_owned(), "f".repeat(64)).is_err());
    }

    #[test]
    fn criterion_values_must_match_the_pinned_source_items() {
        let (input, mut output, _, _) = fixtures();
        let target = output["run_results"].as_array_mut().unwrap().iter_mut()
            .find(|run| optional_string(run, "run_id") == Some("RECON-001-ONE-TO-ONE")).unwrap();
        target["hypotheses"][0]["criterion_evaluations"].as_array_mut().unwrap()
            .iter_mut().find(|evaluation| optional_string(evaluation, "criterion_ref") == Some("CR-AMOUNT")).unwrap()["left_values"] = json!(["999999"]);
        assert!(build_report(&input, &output, output["source_bundle_sha256"].as_str().unwrap().to_owned(), "f".repeat(64)).is_err());
    }


    #[test]
    fn review_action_cannot_be_overridden_by_a_high_weighted_score() {
        let (mut input, mut output, _, _) = fixtures();
        input["runs"][0]["source_items"][1]["transaction_type"] = Value::String("manual_adjustment".to_owned());
        let run = output["run_results"].as_array_mut().unwrap().iter_mut()
            .find(|run| optional_string(run, "run_id") == Some("RECON-001-ONE-TO-ONE")).unwrap();
        let hypothesis = &mut run["hypotheses"][0];
        let evaluation = hypothesis["criterion_evaluations"].as_array_mut().unwrap()
            .iter_mut().find(|evaluation| optional_string(evaluation, "criterion_ref") == Some("CR-TRANSACTION-TYPE")).unwrap();
        evaluation["right_values"] = json!(["manual_adjustment"]);
        evaluation["outcome"] = Value::String("mismatch".to_owned());
        hypothesis["score_bps"] = Value::from(9500);
        assert!(build_report(&input, &output, output["source_bundle_sha256"].as_str().unwrap().to_owned(), "f".repeat(64)).is_err());
    }

    #[test]
    fn review_action_accepts_review_required_even_when_score_exceeds_auto_threshold() {
        let (mut input, mut output, _, _) = fixtures();
        input["runs"][0]["source_items"][1]["transaction_type"] = Value::String("manual_adjustment".to_owned());
        let run = output["run_results"].as_array_mut().unwrap().iter_mut()
            .find(|run| optional_string(run, "run_id") == Some("RECON-001-ONE-TO-ONE")).unwrap();
        let hypothesis = &mut run["hypotheses"][0];
        let evaluation = hypothesis["criterion_evaluations"].as_array_mut().unwrap()
            .iter_mut().find(|evaluation| optional_string(evaluation, "criterion_ref") == Some("CR-TRANSACTION-TYPE")).unwrap();
        evaluation["right_values"] = json!(["manual_adjustment"]);
        evaluation["outcome"] = Value::String("mismatch".to_owned());
        hypothesis["score_bps"] = Value::from(9500);
        hypothesis["decision_state"] = Value::String("review_required".to_owned());
        let report = build_report(&input, &output);
        assert_eq!(run_result(&report, "RECON-001-ONE-TO-ONE")["review_required_count"], 1);
    }

    #[test]
    fn alternative_hypothesis_must_share_items_and_fall_within_ambiguity_margin() {
        let (input, mut output, _, _) = fixtures();
        let target = output["run_results"].as_array_mut().unwrap().iter_mut()
            .find(|run| optional_string(run, "run_id") == Some("RECON-006-AMBIGUOUS-ALTERNATIVES")).unwrap();
        target["hypotheses"][1]["left_item_refs"] = json!(["R6-L1"]);
        target["hypotheses"][1]["right_item_refs"] = json!(["R6-L2"]);
        target["hypotheses"][1]["candidate_pairs"][0]["left_item_ref"] = Value::String("R6-L1".to_owned());
        target["hypotheses"][1]["candidate_pairs"][0]["right_item_ref"] = Value::String("R6-L2".to_owned());
        assert!(build_report(&input, &output, output["source_bundle_sha256"].as_str().unwrap().to_owned(), "f".repeat(64)).is_err());
    }

    #[test]
    fn unexecuted_synthetic_fixture_cannot_claim_an_engine_digest() {
        let (input, mut output, _, _) = fixtures();
        output["matcher_binding"]["engine_sha256"] = Value::String("e".repeat(64));
        assert!(build_report(&input, &output, output["source_bundle_sha256"].as_str().unwrap().to_owned(), "f".repeat(64)).is_err());
    }

    #[test]
    fn reversed_direction_without_reversal_type_is_not_a_reversal_match() {
        let (mut input, output, _, _) = fixtures();
        input["runs"][7]["source_items"][0]["transaction_type"] = Value::String("supplier_payment".to_owned());
        input["runs"][7]["source_items"][0]["reversal_of_external_id"] = Value::Null;
        assert!(validate_input_run(&input["runs"][7]).is_ok());
        let mutated_output = output["run_results"][7]["hypotheses"][0].clone();
        assert_eq!(mutated_output["decision_state"], "eligible_for_auto_match");
        // A complete reversal-lineage mismatch is caught by the selected reversal checks in a pair mutation.
        let mut output_mut = output.clone();
        output_mut["run_results"][7]["hypotheses"][0]["criterion_evaluations"].as_array_mut().unwrap()
            .iter_mut().find(|evaluation| optional_string(evaluation, "criterion_ref") == Some("CR-TRANSACTION-TYPE")).unwrap()["outcome"] = Value::String("mismatch".to_owned());
        assert!(build_report(&input, &output_mut, output_mut["source_bundle_sha256"].as_str().unwrap().to_owned(), "f".repeat(64)).is_err());
    }
}
