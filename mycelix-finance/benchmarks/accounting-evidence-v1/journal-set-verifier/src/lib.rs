#![forbid(unsafe_code)]

use jsonschema::draft202012;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

type AppResult<T> = Result<T, String>;

#[derive(Debug)]
struct Assessment {
    disposition: &'static str,
    reasons: BTreeSet<String>,
    journal_count: usize,
    event_count: usize,
}

pub fn verify(
    candidate_bytes: &[u8],
    candidate_schema_bytes: &[u8],
    oracle_bytes: &[u8],
    oracle_schema_bytes: &[u8],
) -> Value {
    let mut errors = Vec::<String>::new();
    let candidate = parse_json("candidate corpus", candidate_bytes, &mut errors);
    let candidate_schema = parse_json("candidate schema", candidate_schema_bytes, &mut errors);
    let oracle = parse_json("public oracle", oracle_bytes, &mut errors);
    let oracle_schema = parse_json("oracle schema", oracle_schema_bytes, &mut errors);

    if errors.is_empty() {
        errors.extend(validate_instance(
            "candidate corpus",
            &candidate_schema,
            &candidate,
        ));
        errors.extend(validate_instance("public oracle", &oracle_schema, &oracle));
    }

    let mut results = Vec::<Value>::new();
    if errors.is_empty() {
        let input_cases = array(&candidate, "cases");
        let oracle_cases = array(&oracle, "cases");
        let mut oracle_by_id = BTreeMap::<String, &Value>::new();
        for row in oracle_cases {
            let id = string(row, "case_id").unwrap_or("<missing>").to_owned();
            if oracle_by_id.insert(id.clone(), row).is_some() {
                errors.push(format!("duplicate oracle case_id: {id}"));
            }
        }
        let mut seen = BTreeSet::<String>::new();
        for case in input_cases {
            let id = string(case, "case_id").unwrap_or("<missing>").to_owned();
            if !seen.insert(id.clone()) {
                errors.push(format!("duplicate candidate case_id: {id}"));
                continue;
            }
            match (oracle_by_id.get(&id), assess_case(case)) {
                (Some(reference), Ok(assessment)) => {
                    let expected = string(reference, "expected_disposition").unwrap_or("<missing>");
                    let blocks = !matches!(
                        assessment.disposition,
                        "complete_exact_population_and_rollforward"
                            | "complete_with_authorized_nonposting"
                    );
                    let expected_blocks = reference
                        .get("blocks_complete")
                        .and_then(Value::as_bool)
                        .unwrap_or(true);
                    let passed = assessment.disposition == expected && blocks == expected_blocks;
                    results.push(json!({
                        "case_id": id,
                        "actual_disposition": assessment.disposition,
                        "expected_disposition": expected,
                        "passed": passed,
                        "blocks_complete": blocks,
                        "expected_blocks_complete": expected_blocks,
                        "journal_count": assessment.journal_count,
                        "event_count": assessment.event_count,
                        "reason_codes": assessment.reasons.into_iter().collect::<Vec<_>>()
                    }));
                }
                (None, _) => errors.push(format!("candidate case {id} has no public oracle row")),
                (_, Err(error)) => errors.push(format!("case {id}: {error}")),
            }
        }
        for id in oracle_by_id.keys() {
            if !seen.contains(id) {
                errors.push(format!("oracle case {id} has no candidate input"));
            }
        }
    }

    let passed = results.iter().filter(|row| row["passed"] == true).count();
    let status = if errors.is_empty() && !results.is_empty() && passed == results.len() {
        "PASS"
    } else {
        "FAIL"
    };
    let report = json!({
        "report_schema": "fin-journal-set-report.v1",
        "benchmark_id": candidate.get("benchmark_id"),
        "dataset_version": candidate.get("dataset_version"),
        "status": status,
        "claim_scope": "public_synthetic_period_journal_set_and_rollforward_only_not_complete_real_books",
        "digests": {
            "candidate_sha256": sha256_hex(candidate_bytes),
            "candidate_schema_sha256": sha256_hex(candidate_schema_bytes),
            "public_oracle_sha256": sha256_hex(oracle_bytes),
            "oracle_schema_sha256": sha256_hex(oracle_schema_bytes),
            "verifier_source_sha256": sha256_hex(include_bytes!("lib.rs"))
        },
        "case_count": results.len(),
        "passed_case_count": passed,
        "errors": errors,
        "cases": results
    });

    let report_schema: Value = match serde_json::from_str(include_str!("../report.schema.json")) {
        Ok(schema) => schema,
        Err(error) => return fail_report(report, format!("embedded report schema: {error}")),
    };
    let report_errors = validate_instance("generated report", &report_schema, &report);
    if report_errors.is_empty() {
        report
    } else {
        fail_report(report, report_errors.join("; "))
    }
}

fn fail_report(mut report: Value, error: String) -> Value {
    report["status"] = Value::String("FAIL".to_owned());
    if let Some(items) = report["errors"].as_array_mut() {
        items.push(Value::String(error));
    }
    report
}

fn parse_json(label: &str, bytes: &[u8], errors: &mut Vec<String>) -> Value {
    match serde_json::from_slice(bytes) {
        Ok(value) => value,
        Err(error) => {
            errors.push(format!("{label} JSON: {error}"));
            Value::Null
        }
    }
}

fn validate_instance(label: &str, schema: &Value, instance: &Value) -> Vec<String> {
    if let Err(error) = jsonschema::draft202012::meta::validate(schema) {
        return vec![format!("{label}: invalid Draft 2020-12 schema: {error}")];
    }
    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(schema)
    {
        Ok(validator) => validator,
        Err(error) => return vec![format!("{label}: schema compilation failed: {error}")],
    };
    validator
        .iter_errors(instance)
        .map(|error| format!("{label}: {} at {}", error, error.instance_path()))
        .collect()
}

fn array<'a>(value: &'a Value, key: &str) -> &'a [Value] {
    value
        .get(key)
        .and_then(Value::as_array)
        .map(Vec::as_slice)
        .unwrap_or(&[])
}

fn string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn amount(value: &Value) -> AppResult<i128> {
    let text = value
        .as_str()
        .ok_or_else(|| "minor-unit amount must be a canonical integer string".to_owned())?;
    if text.is_empty()
        || (text.len() > 1 && text.starts_with('0'))
        || !text.bytes().all(|byte| byte.is_ascii_digit())
    {
        return Err(format!("non-canonical non-negative amount: {text}"));
    }
    let parsed = text
        .parse::<u128>()
        .map_err(|error| format!("minor-unit amount parse failed: {error}"))?;
    i128::try_from(parsed).map_err(|_| "minor-unit amount exceeds i128 range".to_owned())
}

fn amount_field(value: &Value, key: &str) -> AppResult<i128> {
    amount(
        value
            .get(key)
            .ok_or_else(|| format!("missing amount field {key}"))?,
    )
}

fn assess_case(case: &Value) -> AppResult<Assessment> {
    let manifest = case
        .get("manifest")
        .ok_or_else(|| "missing manifest".to_owned())?;
    let scope = manifest
        .get("scope")
        .ok_or_else(|| "missing scope".to_owned())?;
    let source = manifest
        .get("source_population")
        .ok_or_else(|| "missing source population".to_owned())?;
    let mut reasons = BTreeSet::<String>::new();
    let mut findings = BTreeSet::<String>::new();

    let entity = string(scope, "reporting_entity").unwrap_or_default();
    let ledger = string(scope, "ledger_id").unwrap_or_default();
    let accounting_profile = string(scope, "accounting_profile_ref").unwrap_or_default();
    let denomination = string(scope, "denomination_profile_ref").unwrap_or_default();
    let period = string(scope, "period_id").unwrap_or_default();
    let close_cutoff = string(scope, "close_cutoff").unwrap_or_default();

    let expected_events = array(source, "expected_event_refs")
        .iter()
        .filter_map(Value::as_str)
        .map(str::to_owned)
        .collect::<BTreeSet<_>>();
    if string(source, "qualification_state") != Some("complete") {
        findings.insert("source_frontier_unqualified".to_owned());
        reasons.insert("expected_population_not_qualified_complete".to_owned());
    }
    if string(source, "expected_event_commitment") != string(source, "closed_event_commitment") {
        findings.insert("stale_close_frontier".to_owned());
        reasons.insert("source_population_frontier_changed_after_close".to_owned());
    }

    let mut event_by_id = BTreeMap::<String, &Value>::new();
    for event in array(manifest, "events") {
        let id = string(event, "event_ref").unwrap_or_default().to_owned();
        if event_by_id.insert(id.clone(), event).is_some() {
            findings.insert("source_frontier_unqualified".to_owned());
            reasons.insert(format!("duplicate_event_record:{id}"));
        }
    }
    if event_by_id.keys().cloned().collect::<BTreeSet<_>>() != expected_events {
        findings.insert("source_frontier_unqualified".to_owned());
        reasons.insert("event_records_do_not_equal_expected_population".to_owned());
    }

    let mut account_by_id = BTreeMap::<String, &Value>::new();
    for account in array(manifest, "account_profiles") {
        let id = string(account, "account_ref")
            .unwrap_or_default()
            .to_owned();
        if account_by_id.insert(id.clone(), account).is_some() {
            findings.insert("unsupported_profile_or_unit".to_owned());
            reasons.insert(format!("duplicate_account_profile:{id}"));
        }
    }

    // Canonicalize physical journal replay: identical identity and semantics count once.
    // Reusing an identity with any changed semantic field is a conflict.
    let mut journal_by_id = BTreeMap::<String, &Value>::new();
    let mut active_journals = Vec::<&Value>::new();
    for journal in array(manifest, "journals") {
        let id = string(journal, "journal_id").unwrap_or_default().to_owned();
        match journal_by_id.get(&id) {
            None => {
                journal_by_id.insert(id, journal);
                active_journals.push(journal);
            }
            Some(previous) => {
                if previous != &journal {
                    findings.insert("conflicting_journal_identity".to_owned());
                    reasons.insert("same_journal_id_changed_semantics".to_owned());
                } else {
                    reasons.insert("exact_journal_replay_idempotent".to_owned());
                }
            }
        }
    }

    let mut journal_by_event = BTreeMap::<String, Vec<&Value>>::new();
    for journal in &active_journals {
        let event_ref = string(journal, "economic_event_ref").unwrap_or_default();
        journal_by_event
            .entry(event_ref.to_owned())
            .or_default()
            .push(*journal);
        if !expected_events.contains(event_ref) {
            findings.insert("orphan_or_unauthorized_journal".to_owned());
            reasons.insert("journal_event_not_in_expected_population".to_owned());
        }
        if string(journal, "entity_ref") != Some(entity)
            || string(journal, "ledger_id") != Some(ledger)
        {
            findings.insert("orphan_or_unauthorized_journal".to_owned());
            reasons.insert("journal_entity_scope_mismatch".to_owned());
        }
        if string(journal, "accounting_profile_ref") != Some(accounting_profile)
            || string(journal, "denomination_profile_ref") != Some(denomination)
            || string(journal, "period_id") != Some(period)
        {
            findings.insert("unsupported_profile_or_unit".to_owned());
            reasons.insert("journal_profile_or_period_mismatch".to_owned());
        }
        let effective_at = string(journal, "effective_at").unwrap_or_default();
        let booking_at = string(journal, "booking_at").unwrap_or_default();
        let post_close_authorized = journal
            .get("post_close_authorized")
            .and_then(Value::as_bool)
            == Some(true);
        let reopen_authorization = string(journal, "post_close_authorization_ref")
            .filter(|value| !value.trim().is_empty());
        let qualified_reopen = post_close_authorized && reopen_authorization.is_some();
        if post_close_authorized && reopen_authorization.is_none() {
            findings.insert("orphan_or_unauthorized_journal".to_owned());
            reasons.insert("post_close_authorization_reference_missing".to_owned());
        }
        if !close_cutoff.is_empty() && effective_at > close_cutoff && !qualified_reopen {
            findings.insert("stale_close_frontier".to_owned());
            reasons.insert("journal_effective_after_close_cutoff".to_owned());
        }
        if !close_cutoff.is_empty() && booking_at > close_cutoff && !qualified_reopen {
            findings.insert("stale_close_frontier".to_owned());
            reasons.insert("journal_received_after_close_without_reopen_authority".to_owned());
        }

        let mut debits = 0_i128;
        let mut credits = 0_i128;
        for posting in array(journal, "postings") {
            let account_ref = string(posting, "account_ref").unwrap_or_default();
            let account = account_by_id.get(account_ref);
            let account = match account {
                Some(account) => *account,
                None => {
                    findings.insert("unsupported_profile_or_unit".to_owned());
                    reasons.insert("posting_account_not_in_pinned_account_profile".to_owned());
                    continue;
                }
            };
            if string(posting, "unit_profile_ref") != Some(denomination)
                || string(account, "unit_profile_ref") != Some(denomination)
            {
                findings.insert("unsupported_profile_or_unit".to_owned());
                reasons.insert("posting_unit_profile_mismatch".to_owned());
            }
            let value = amount_field(posting, "amount_minor")?;
            match string(posting, "direction") {
                Some("DEBIT") => {
                    debits = debits
                        .checked_add(value)
                        .ok_or_else(|| "debit sum overflow".to_owned())?;
                }
                Some("CREDIT") => {
                    credits = credits
                        .checked_add(value)
                        .ok_or_else(|| "credit sum overflow".to_owned())?;
                }
                _ => {
                    findings.insert("unsupported_profile_or_unit".to_owned());
                    reasons.insert("invalid_posting_direction".to_owned());
                }
            }
        }
        if debits != credits {
            findings.insert("orphan_or_unauthorized_journal".to_owned());
            reasons.insert("journal_entry_not_balanced".to_owned());
        }
    }

    for (event_ref, journals) in &journal_by_event {
        if journals.len() > 1 {
            findings.insert("duplicate_economic_posting".to_owned());
            reasons.insert(format!(
                "economic_event_projected_more_than_once:{event_ref}"
            ));
        }
    }

    for event_ref in &expected_events {
        let event = event_by_id
            .get(event_ref)
            .ok_or_else(|| format!("expected event {event_ref} has no event disposition"))?;
        let treatment = string(event, "terminal_treatment").unwrap_or_default();
        let matching = journal_by_event
            .get(event_ref)
            .map(Vec::as_slice)
            .unwrap_or(&[]);
        match treatment {
            "journal_required" if matching.len() == 0 => {
                findings.insert("missing_journal_projection".to_owned());
                reasons.insert(format!("required_event_without_journal:{event_ref}"));
            }
            "journal_required" if matching.len() > 1 => {}
            "journal_required" => {}
            "authorized_nonposting" if matching.is_empty() => {
                let authorized = string(event, "authorization_ref")
                    .map(|value| !value.trim().is_empty())
                    .unwrap_or(false)
                    && string(event, "reason_ref")
                        .map(|value| !value.trim().is_empty())
                        .unwrap_or(false);
                if !authorized {
                    findings.insert("missing_journal_projection".to_owned());
                    reasons.insert(format!("nonposting_disposition_not_authorized:{event_ref}"));
                } else {
                    reasons.insert(format!("authorized_nonposting_treatment:{event_ref}"));
                }
            }
            "authorized_nonposting" => {
                findings.insert("orphan_or_unauthorized_journal".to_owned());
                reasons.insert(format!("nonposting_event_has_journal:{event_ref}"));
            }
            _ => {
                findings.insert("missing_journal_projection".to_owned());
                reasons.insert(format!("unresolved_event_cannot_be_excluded:{event_ref}"));
            }
        }
    }

    // Corrections and reversals are append-only. Reversal capacity is accumulated
    // across every reversal of one original journal, never reset per reversal.
    let mut reversal_remaining =
        BTreeMap::<String, BTreeMap<(String, String), i128>>::new();
    for journal in &active_journals {
        let reversal_of = string(journal, "reversal_of");
        let correction_of = string(journal, "correction_of");
        if let Some(target_id) = reversal_of.or(correction_of) {
            let original = journal_by_id.get(target_id);
            let original = match original {
                Some(original) if *original != *journal => *original,
                _ => {
                    findings.insert("orphan_or_unauthorized_journal".to_owned());
                    reasons.insert("correction_original_not_present".to_owned());
                    continue;
                }
            };
            if reversal_of.is_some() {
                if string(original, "reversal_of").is_some()
                    || string(original, "correction_of").is_some()
                {
                    findings.insert("orphan_or_unauthorized_journal".to_owned());
                    reasons.insert("reversal_target_is_not_an_original_journal".to_owned());
                    continue;
                }

                if !reversal_remaining.contains_key(target_id) {
                    let mut capacity = BTreeMap::<(String, String), i128>::new();
                    for posting in array(original, "postings") {
                        let key = (
                            string(posting, "account_ref").unwrap_or_default().to_owned(),
                            string(posting, "direction").unwrap_or_default().to_owned(),
                        );
                        let value = amount_field(posting, "amount_minor")?;
                        let total = capacity.entry(key).or_default();
                        match total.checked_add(value) {
                            Some(updated) => *total = updated,
                            None => {
                                findings.insert("orphan_or_unauthorized_journal".to_owned());
                                reasons.insert("original_posting_capacity_overflow".to_owned());
                            }
                        }
                    }
                    reversal_remaining.insert(target_id.to_owned(), capacity);
                }

                let remaining = reversal_remaining
                    .get_mut(target_id)
                    .expect("capacity inserted for original journal");
                for posting in array(journal, "postings") {
                    let account = string(posting, "account_ref")
                        .unwrap_or_default()
                        .to_owned();
                    let original_direction = match string(posting, "direction") {
                        Some("DEBIT") => "CREDIT",
                        Some("CREDIT") => "DEBIT",
                        _ => {
                            findings.insert("orphan_or_unauthorized_journal".to_owned());
                            reasons.insert("reversal_has_invalid_posting_direction".to_owned());
                            continue;
                        }
                    };
                    let key = (account, original_direction.to_owned());
                    let value = amount_field(posting, "amount_minor")?;
                    match remaining.get_mut(&key) {
                        Some(limit) if value <= *limit => *limit -= value,
                        _ => {
                            findings.insert("orphan_or_unauthorized_journal".to_owned());
                            reasons.insert("reversal_exceeds_original_posting".to_owned());
                        }
                    }
                }
            }
        }
    }

    // Prior-period close must equal this period's opening account balances exactly.
    if manifest
        .get("prior_close")
        .and_then(|value| value.get("present"))
        .and_then(Value::as_bool)
        != Some(true)
    {
        findings.insert("period_boundary_conflict".to_owned());
        reasons.insert("predecessor_close_missing".to_owned());
    }
    let empty_prior_close = Value::Null;
    let prior = array(
        manifest.get("prior_close").unwrap_or(&empty_prior_close),
        "account_balances",
    );
    let opening = array(manifest, "opening_balances");
    let prior_map = balance_map(prior)?;
    let opening_map = balance_map(opening)?;
    if prior_map != opening_map {
        findings.insert("period_boundary_conflict".to_owned());
        reasons.insert("opening_balance_differs_from_prior_close".to_owned());
    }

    // Reconstruct every account balance from opening balance plus signed postings.
    let mut reconstructed = opening_map.clone();
    for journal in &active_journals {
        for posting in array(journal, "postings") {
            let account_ref = string(posting, "account_ref")
                .unwrap_or_default()
                .to_owned();
            let profile = account_by_id.get(&account_ref);
            let profile = match profile {
                Some(profile) => *profile,
                None => continue,
            };
            let normal_side = string(profile, "normal_side").unwrap_or_default();
            let direction = string(posting, "direction").unwrap_or_default();
            let value = amount_field(posting, "amount_minor")?;
            let signed = if direction == normal_side {
                value
            } else {
                -value
            };
            let current = reconstructed.entry(account_ref.clone()).or_default();
            *current = current
                .checked_add(signed)
                .ok_or_else(|| format!("account balance overflow for {account_ref}"))?;
        }
    }
    let closing_map = balance_map(array(manifest, "closing_balances"))?;
    if reconstructed != closing_map {
        findings.insert("account_rollforward_conflict".to_owned());
        reasons.insert("closing_balance_differs_from_reconstructed_account".to_owned());
    }

    // Closing trial balance must balance across the declared account normal sides.
    let mut debit_total = 0_i128;
    let mut credit_total = 0_i128;
    for (account_ref, value) in &closing_map {
        let profile = account_by_id.get(account_ref);
        let profile = match profile {
            Some(profile) => *profile,
            None => {
                findings.insert("unsupported_profile_or_unit".to_owned());
                reasons.insert("closing_balance_account_not_in_pinned_profile".to_owned());
                continue;
            }
        };
        match string(profile, "normal_side") {
            Some("DEBIT") => {
                debit_total = debit_total
                    .checked_add(*value)
                    .ok_or_else(|| "trial balance debit overflow".to_owned())?
            }
            Some("CREDIT") => {
                credit_total = credit_total
                    .checked_add(*value)
                    .ok_or_else(|| "trial balance credit overflow".to_owned())?
            }
            _ => {
                findings.insert("unsupported_profile_or_unit".to_owned());
                reasons.insert("invalid_account_normal_side".to_owned());
            }
        }
    }
    if debit_total != credit_total {
        findings.insert("account_rollforward_conflict".to_owned());
        reasons.insert("closing_trial_balance_not_balanced".to_owned());
    }

    let disposition = if findings.contains("conflicting_journal_identity") {
        "conflicting_journal_identity"
    } else if findings.contains("duplicate_economic_posting") {
        "duplicate_economic_posting"
    } else if findings.contains("missing_journal_projection") {
        "missing_journal_projection"
    } else if findings.contains("orphan_or_unauthorized_journal") {
        "orphan_or_unauthorized_journal"
    } else if findings.contains("period_boundary_conflict") {
        "period_boundary_conflict"
    } else if findings.contains("unsupported_profile_or_unit") {
        "unsupported_profile_or_unit"
    } else if findings.contains("source_frontier_unqualified") {
        "source_frontier_unqualified"
    } else if findings.contains("stale_close_frontier") {
        "stale_close_frontier"
    } else if findings.contains("account_rollforward_conflict") {
        "account_rollforward_conflict"
    } else if expected_events.iter().any(|event| {
        event_by_id
            .get(event)
            .map(|record| string(record, "terminal_treatment") == Some("authorized_nonposting"))
            .unwrap_or(false)
    }) {
        "complete_with_authorized_nonposting"
    } else {
        "complete_exact_population_and_rollforward"
    };

    Ok(Assessment {
        disposition,
        reasons,
        journal_count: active_journals.len(),
        event_count: expected_events.len(),
    })
}

fn balance_map(rows: &[Value]) -> AppResult<BTreeMap<String, i128>> {
    let mut values = BTreeMap::<String, i128>::new();
    for row in rows {
        let account = string(row, "account_ref")
            .ok_or_else(|| "balance missing account_ref".to_owned())?
            .to_owned();
        let value = amount_field(row, "amount_minor")?;
        if values.insert(account.clone(), value).is_some() {
            return Err(format!("duplicate account balance: {account}"));
        }
    }
    Ok(values)
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    fn parse(source: &str) -> Value {
        serde_json::from_str(source).expect("valid fixture JSON")
    }

    fn case_by_id<'a>(candidate: &'a Value, id: &str) -> &'a Value {
        candidate["cases"]
            .as_array()
            .expect("candidate cases")
            .iter()
            .find(|row| row["case_id"] == id)
            .expect("case exists")
    }

    #[test]
    fn all_public_reference_cases_match_the_disposition_oracle() {
        let candidate = parse(include_str!("../../fin_journal_set_candidates_v1.1.0.json"));
        let oracle = parse(include_str!(
            "../../public-journal-set-oracle/fin_journal_set_oracle_v1.1.0.json"
        ));
        for case in candidate["cases"].as_array().expect("cases") {
            let id = case["case_id"].as_str().expect("case ID");
            let expected = oracle["cases"]
                .as_array()
                .expect("oracle cases")
                .iter()
                .find(|row| row["case_id"] == id)
                .expect("oracle row")["expected_disposition"]
                .as_str()
                .expect("expected disposition");
            assert_eq!(
                assess_case(case).expect("assessment").disposition,
                expected,
                "{id}"
            );
        }
    }

    #[test]
    fn an_individually_balanced_journal_does_not_prove_population_completeness() {
        let candidate = parse(include_str!("../../fin_journal_set_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FJS-002");
        assert_eq!(
            assess_case(case).expect("assessment").disposition,
            "missing_journal_projection"
        );
    }

    #[test]
    fn balanced_orphan_entry_is_not_admitted() {
        let candidate = parse(include_str!("../../fin_journal_set_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FJS-005");
        assert_eq!(
            assess_case(case).expect("assessment").disposition,
            "orphan_or_unauthorized_journal"
        );
    }

    #[test]
    fn exact_journal_replay_is_idempotent() {
        let candidate = parse(include_str!("../../fin_journal_set_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FJS-006");
        assert_eq!(
            assess_case(case).expect("assessment").disposition,
            "complete_exact_population_and_rollforward"
        );
    }

    #[test]
    fn a_duplicate_economic_event_is_rejected_even_when_both_entries_balance() {
        let candidate = parse(include_str!("../../fin_journal_set_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FJS-004");
        assert_eq!(
            assess_case(case).expect("assessment").disposition,
            "duplicate_economic_posting"
        );
    }

    #[test]
    fn account_rollforward_uses_exact_minor_units() {
        let candidate = parse(include_str!("../../fin_journal_set_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FJS-013");
        assert_eq!(
            assess_case(case).expect("assessment").disposition,
            "account_rollforward_conflict"
        );
    }

    #[test]
    fn a_reversal_must_be_append_only_and_bounded_by_original_postings() {
        let candidate = parse(include_str!("../../fin_journal_set_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FJS-018");
        assert_eq!(
            assess_case(case).expect("assessment").disposition,
            "orphan_or_unauthorized_journal"
        );
    }

    #[test]
    fn source_frontier_change_stales_a_frozen_close() {
        let candidate = parse(include_str!("../../fin_journal_set_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FJS-022");
        assert_eq!(
            assess_case(case).expect("assessment").disposition,
            "stale_close_frontier"
        );
    }

    #[test]
    fn semantically_equivalent_permutations_preserve_disposition() {
        let candidate = parse(include_str!("../../fin_journal_set_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FJS-023");
        assert_eq!(
            assess_case(case).expect("assessment").disposition,
            "complete_exact_population_and_rollforward"
        );
    }

    #[test]
    fn post_close_flag_without_authorization_reference_is_rejected() {
        let candidate = parse(include_str!("../../fin_journal_set_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FJS-025");
        let assessment = assess_case(case).expect("assessment");
        assert_eq!(assessment.disposition, "orphan_or_unauthorized_journal");
        assert!(
            assessment
                .reasons
                .contains("post_close_authorization_reference_missing")
        );
    }

    #[test]
    fn multiple_partial_reversals_cannot_exceed_the_original_in_aggregate() {
        let candidate = parse(include_str!("../../fin_journal_set_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FJS-026");
        let assessment = assess_case(case).expect("assessment");
        assert_eq!(assessment.disposition, "orphan_or_unauthorized_journal");
        assert!(assessment.reasons.contains("reversal_exceeds_original_posting"));
    }
}
