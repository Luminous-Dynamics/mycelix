#![forbid(unsafe_code)]

//! Synthetic-only evaluator for purpose-bound payment fraud assertions.
//! A signal may support a bounded advisory action; it cannot adjudicate guilt,
//! authorize payment execution, or supply a private platform's legal basis.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const CORPUS_DEFAULT: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-fraud-signal-corpus-v1.json");
const SCHEMA_DEFAULT: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-fraud-signal-corpus-v1.schema.json");

fn str_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn u64_at(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn array_at<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn contains_string(value: &Value, key: &str, expected: &str) -> bool {
    array_at(value, key)
        .is_some_and(|items| items.iter().any(|item| item.as_str() == Some(expected)))
}

fn utc_epoch_seconds(value: &str) -> Option<i64> {
    let value = value.strip_suffix('Z')?;
    let (date, time) = value.split_once('T')?;
    let mut date_parts = date.split('-');
    let year = date_parts.next()?.parse::<i64>().ok()?;
    let month = date_parts.next()?.parse::<i64>().ok()?;
    let day = date_parts.next()?.parse::<i64>().ok()?;
    if date_parts.next().is_some() || year < 1970 || !(1..=12).contains(&month) {
        return None;
    }
    let leap_year = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days_in_month = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap_year => 29,
        2 => 28,
        _ => return None,
    };
    if !(1..=days_in_month).contains(&day) {
        return None;
    }

    let mut time_parts = time.split(':');
    let hour = time_parts.next()?.parse::<i64>().ok()?;
    let minute = time_parts.next()?.parse::<i64>().ok()?;
    let second = time_parts.next()?.parse::<i64>().ok()?;
    if time_parts.next().is_some()
        || !(0..=23).contains(&hour)
        || !(0..=59).contains(&minute)
        || !(0..=59).contains(&second)
    {
        return None;
    }

    // All fixture timestamps are canonical UTC seconds ending in Z. Reject
    // offsets/fractions so lexical source fields cannot create clock ambiguity.
    let adjusted_year = year - i64::from(month <= 2);
    let era = adjusted_year.div_euclid(400);
    let year_of_era = adjusted_year - era * 400;
    let adjusted_month = month + if month > 2 { -3 } else { 9 };
    let day_of_year = (153 * adjusted_month + 2) / 5 + day - 1;
    let day_of_era =
        year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    let days_since_epoch = era * 146_097 + day_of_era - 719_468;
    Some(days_since_epoch * 86_400 + hour * 3_600 + minute * 60 + second)
}

fn unique_string_count(value: &Value, key: &str) -> usize {
    array_at(value, key)
        .map(|items| {
            items
                .iter()
                .filter_map(Value::as_str)
                .collect::<BTreeSet<_>>()
                .len()
        })
        .unwrap_or(0)
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

fn contains_global_subject_ref(query: &Value) -> bool {
    str_at(query, "recipient_subject_ref").is_some_and(|value| value.starts_with("global:"))
}

fn decision(signal: &Value, query: &Value, profile: &Value) -> (&'static str, &'static str) {
    let sharing = &signal["sharing"];
    let issuer = &signal["issuer"];
    let verification = &issuer["verification"];
    let subject = &signal["subject"];
    let assertion = &signal["assertion"];
    let lifecycle = &signal["lifecycle"];
    let event = &signal["observed_event"];

    let basis = str_at(sharing, "lawful_basis_code").unwrap_or("unknown");
    let profile_state = str_at(profile, "operational_state").unwrap_or("unknown");
    let category = str_at(event, "category").unwrap_or("unknown");
    let status = str_at(assertion, "status").unwrap_or("unknown");
    let evidence_class = str_at(assertion, "evidence_class").unwrap_or("unknown");
    let data_class = str_at(subject, "data_classification").unwrap_or("unknown");
    let issuer_role = str_at(issuer, "issuer_role").unwrap_or("unknown");
    let controller_role = str_at(issuer, "controller_role").unwrap_or("unknown");
    let recipient = str_at(query, "recipient_id").unwrap_or("");
    let recipient_class = str_at(query, "recipient_class").unwrap_or("");
    let purpose = str_at(query, "requested_purpose").unwrap_or("");
    let action = str_at(query, "requested_action").unwrap_or("");
    let now = str_at(query, "evaluated_at").unwrap_or("");
    let expiry = str_at(sharing, "retention_expires_at").unwrap_or("");
    let issuer_id = str_at(issuer, "participant_id").unwrap_or("");

    if !contains_string(profile, "allowed_issuer_ids", issuer_id) {
        return ("reject", "issuer_not_permitted_by_profile");
    }
    if !contains_string(profile, "allowed_assertion_statuses", status) {
        return ("reject", "assertion_status_not_permitted_by_profile");
    }

    // Comparator and programme contexts are deliberately narrower than
    // transaction-level fraud assertions.
    if profile_state == "knowledge_exchange_only"
        && (data_class != "aggregate_non_personal" || category != "programme_status_observation")
    {
        return ("reject", "knowledge_forum_not_customer_level_exchange");
    }
    if category == "payee_name_match_result" {
        return ("reject", "payee_match_not_fraud_adjudication");
    }
    if category == "reimbursement_outcome" {
        return (
            "reject",
            "aggregate_reimbursement_outcome_not_live_fraud_signal",
        );
    }

    if action == "automatic_irreversible_denial" {
        return (
            "reject",
            "single_signal_cannot_authorize_irreversible_denial",
        );
    }
    if action == "execute_payment" {
        return ("reject", "fraud_signal_cannot_authorize_payment_execution");
    }
    if matches!(action, "credit_scoring" | "marketing" | "legal_guilt_label")
        || contains_string(sharing, "prohibited_purposes", purpose)
    {
        return ("reject", "prohibited_secondary_purpose");
    }
    if contains_global_subject_ref(query) {
        return ("reject", "global_subject_identifier");
    }
    let pairwise_prefix = format!("pairwise:{recipient}:");
    if !str_at(query, "recipient_subject_ref")
        .is_some_and(|value| value.starts_with(&pairwise_prefix))
    {
        return ("reject", "pairwise_subject_scope_mismatch");
    }
    match str_at(query, "subject_binding_state") {
        Some("mismatch") => return ("reject", "subject_binding_mismatch"),
        Some("unknown") => return ("quarantine", "subject_binding_unknown"),
        Some("verified") if str_at(query, "subject_binding_receipt_ref").is_some() => {}
        _ => return ("quarantine", "subject_binding_unknown"),
    }

    if basis == "FSR_ACT_251_REGULATOR_SCOPE"
        && profile_state != "synthetic_statutory_scope_fixture_only"
    {
        return ("reject", "section_251_not_private_platform_basis");
    }

    if profile_state == "synthetic_statutory_scope_fixture_only" {
        if basis != "FSR_ACT_251_REGULATOR_SCOPE"
            || !matches!(issuer_role, "financial_sector_regulator" | "reserve_bank")
            || !matches!(
                controller_role,
                "financial_sector_regulator" | "reserve_bank"
            )
        {
            return ("reject", "section_251_issuer_scope_mismatch");
        }
        if !contains_string(profile, "allowed_recipient_classes", recipient_class)
            || !contains_string(profile, "allowed_purposes", purpose)
            || !purpose.starts_with("statutory_fraud_")
            || !contains_string(sharing, "permitted_recipient_ids", recipient)
        {
            return ("reject", "section_251_recipient_scope_mismatch");
        }
    } else if !contains_string(profile, "controller_roles", controller_role) {
        return ("reject", "issuer_controller_role_not_in_profile");
    }

    if str_at(profile, "profile_id") == Some("ZA-PEM-FRAUD-NPU-PROPOSAL-V1")
        || str_at(profile, "profile_id") == Some("ZA-PEM-FRAUD-FORUM-KNOWLEDGE-ONLY-V1")
    {
        if data_class == "personal" || data_class == "special_personal_alleged_offence" {
            return (
                "reject",
                "programme_status_is_not_person_level_exchange_authority",
            );
        }
        if profile_state == "proposed_or_being_scoped" {
            return ("quarantine", "proposed_programme_not_operational");
        }
    }

    if str_at(verification, "signature_state") == Some("invalid") {
        return ("reject", "invalid_signature");
    }
    if str_at(verification, "key_state") == Some("revoked") {
        return ("reject", "revoked_signing_key");
    }
    if str_at(verification, "signature_state") != Some("verified")
        || str_at(verification, "key_state") != Some("valid")
    {
        return ("quarantine", "unknown_signing_key");
    }
    if str_at(verification, "payload_digest") != str_at(verification, "verified_payload_digest") {
        return ("reject", "verification_receipt_payload_mismatch");
    }
    if profile_state == "proposed_or_being_scoped" {
        return ("quarantine", "proposed_programme_not_operational");
    }
    if str_at(issuer, "authority_scope_state") == Some("denied") {
        return ("reject", "issuer_authority_denied");
    }
    if str_at(issuer, "authority_scope_state") != Some("authorized") {
        return ("quarantine", "issuer_authority_unknown");
    }

    if str_at(sharing, "storage_scope") == Some("public_dht")
        && matches!(data_class, "personal" | "special_personal_alleged_offence")
    {
        return ("reject", "personal_signal_in_public_dht");
    }
    if str_at(sharing, "storage_scope") == Some("unknown")
        && matches!(data_class, "personal" | "special_personal_alleged_offence")
    {
        return ("quarantine", "personal_storage_scope_unknown");
    }
    if data_class == "unknown" {
        return ("quarantine", "data_classification_unknown");
    }
    if str_at(sharing, "lawful_basis_status") == Some("denied") {
        return ("reject", "lawful_basis_denied");
    }
    if str_at(sharing, "lawful_basis_status") != Some("reviewed_for_fixture")
        || str_at(sharing, "lawful_basis_ref").is_none()
    {
        if data_class == "personal" || data_class == "special_personal_alleged_offence" {
            return ("quarantine", "personal_data_basis_unknown");
        }
        return ("quarantine", "lawful_basis_unknown");
    }
    if data_class == "special_personal_alleged_offence" {
        match str_at(sharing, "special_info_review_status") {
            Some("denied") => return ("reject", "special_personal_review_denied"),
            Some("reviewed_for_fixture")
                if str_at(sharing, "special_info_review_ref").is_some() => {}
            _ => return ("quarantine", "special_personal_review_pending"),
        }
    }

    if str_at(sharing, "retention_expires_at").is_none() || str_at(query, "evaluated_at").is_none()
    {
        return ("quarantine", "retention_or_evaluation_time_unknown");
    }
    if expiry <= now {
        return ("quarantine", "retention_expired");
    }

    let Some(observed_at) = str_at(event, "observed_at").and_then(utc_epoch_seconds) else {
        return ("quarantine", "signal_time_unqualified");
    };
    let Some(recorded_at) = str_at(event, "recorded_at").and_then(utc_epoch_seconds) else {
        return ("quarantine", "signal_time_unqualified");
    };
    let Some(evaluated_at) = utc_epoch_seconds(now) else {
        return ("quarantine", "signal_time_unqualified");
    };
    if observed_at > recorded_at {
        return ("quarantine", "signal_time_order_invalid");
    }
    if observed_at > evaluated_at || recorded_at > evaluated_at {
        return ("quarantine", "signal_event_from_future");
    }
    let Some(max_signal_age) = u64_at(profile, "max_signal_age_seconds") else {
        return ("quarantine", "signal_freshness_policy_unknown");
    };
    if evaluated_at - observed_at > max_signal_age as i64 {
        return ("quarantine", "signal_stale");
    }

    if str_at(query, "recipient_id").is_none() || str_at(query, "recipient_class").is_none() {
        return ("reject", "recipient_identity_missing");
    }
    if !contains_string(sharing, "permitted_recipient_ids", recipient) {
        return ("reject", "recipient_not_permitted");
    }
    if !contains_string(profile, "allowed_recipient_classes", recipient_class) {
        return ("reject", "recipient_class_not_permitted");
    }
    if !contains_string(profile, "allowed_signal_categories", category) {
        return ("reject", "signal_category_not_in_profile");
    }
    if !contains_string(profile, "allowed_data_classes", data_class) {
        return ("reject", "data_class_not_in_profile");
    }
    if !contains_string(profile, "allowed_purposes", purpose)
        || !contains_string(sharing, "permitted_purposes", purpose)
    {
        return ("reject", "purpose_not_permitted");
    }
    if !contains_string(profile, "permitted_actions", action) {
        return ("reject", "action_not_permitted_by_profile");
    }

    let correction_status = str_at(lifecycle, "correction_status").unwrap_or("unknown");
    let previous = u64_at(lifecycle, "previous_recipient_count").unwrap_or(u64::MAX);
    let notified = u64_at(lifecycle, "notified_recipient_count").unwrap_or(0);
    let previous_ids = array_at(lifecycle, "previous_recipient_ids");
    let notified_ids = array_at(lifecycle, "notified_recipient_ids");
    let previous_ids_len = previous_ids.map_or(0, Vec::len);
    let notified_ids_len = notified_ids.map_or(0, Vec::len);
    if previous != previous_ids_len as u64 || notified != notified_ids_len as u64 {
        return ("quarantine", "correction_recipient_ledger_mismatch");
    }
    if notified_ids.is_some_and(|notified_items| {
        notified_items.iter().any(|recipient_id| {
            !previous_ids.is_some_and(|previous_items| previous_items.contains(recipient_id))
        })
    }) {
        return ("quarantine", "correction_recipient_ledger_mismatch");
    }

    let appeal_status = str_at(lifecycle, "appeal_status").unwrap_or("unknown");
    let appeal_record = str_at(lifecycle, "appeal_record_ref");
    let appeal_decision = str_at(lifecycle, "appeal_decision_ref");
    match appeal_status {
        "none" if appeal_record.is_some() || appeal_decision.is_some() => {
            return ("quarantine", "appeal_reference_without_case_state");
        }
        "open" if appeal_record.is_none() || appeal_decision.is_some() => {
            return ("quarantine", "appeal_record_missing");
        }
        "accepted" | "rejected" | "closed"
            if appeal_record.is_none() || appeal_decision.is_none() =>
        {
            return ("quarantine", "appeal_disposition_missing");
        }
        "none" | "open" | "accepted" | "rejected" | "closed" => {}
        _ => return ("quarantine", "appeal_state_unknown"),
    }

    if correction_status != "none"
        && array_at(lifecycle, "correction_record_refs").is_none_or(Vec::is_empty)
    {
        return ("quarantine", "correction_record_missing");
    }
    if correction_status == "correction_applied"
        && str_at(lifecycle, "superseding_signal_id").is_none()
    {
        return ("quarantine", "superseding_signal_missing");
    }
    if correction_status == "contested" || status == "disputed" {
        return ("quarantine", "accuracy_contested");
    }
    if matches!(
        correction_status,
        "correction_applied" | "withdrawn" | "superseded"
    ) && previous_ids.is_some_and(|ids| {
        ids.iter()
            .any(|id| !notified_ids.is_some_and(|delivered| delivered.contains(id)))
    }) {
        return ("quarantine", "correction_not_propagated");
    }
    if status == "corrected" || correction_status == "superseded" {
        return ("quarantine", "superseded_assertion_not_current");
    }
    if status == "withdrawn" || correction_status == "withdrawn" {
        return ("quarantine", "withdrawn_assertion_not_current");
    }
    if status == "not_substantiated" {
        return ("quarantine", "not_substantiated_assertion_not_current");
    }

    if status == "corroborated" && unique_string_count(assertion, "evidence_group_ids") < 2 {
        return ("reject", "correlated_copies_not_independent");
    }
    if status == "official_determination" && evidence_class == "model_derived_indicator" {
        return ("reject", "model_output_not_official_determination");
    }
    if status == "official_determination"
        && (evidence_class != "court_or_official_record"
            || str_at(assertion, "determining_authority_ref").is_none())
    {
        return ("reject", "official_status_missing_authority_evidence");
    }
    if str_at(assertion, "confidence_level") == Some("high")
        && str_at(assertion, "confidence_method_ref").is_none()
    {
        return ("reject", "high_confidence_without_method");
    }

    if profile_state == "synthetic_statutory_scope_fixture_only" {
        return ("admit_advisory", "regulator_statutory_scope_fixture");
    }
    if data_class == "special_personal_alleged_offence" {
        return ("admit_advisory", "reviewed_fixture_scope_only");
    }
    ("admit_advisory", "valid_scoped_transaction_signal")
}

fn semantic_errors(corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if str_at(corpus, "schema") != Some("mycelix.sa-payment-fraud-signal/v1")
        || str_at(corpus, "corpus_id") != Some("ZA-SA-PAYMENT-FRAUD-SIGNAL-SYNTHETIC-V1")
        || str_at(corpus, "status") != Some("synthetic_contract_fixture_only")
        || corpus
            .get("external_effects_enabled")
            .and_then(Value::as_bool)
            != Some(false)
    {
        errors.push("corpus identity/status/effects boundary invalid".into());
    }

    let sources = array_at(corpus, "source_register")
        .cloned()
        .unwrap_or_default();
    let profiles = array_at(corpus, "profiles").cloned().unwrap_or_default();
    let scenarios = array_at(corpus, "scenarios").cloned().unwrap_or_default();

    let mut source_ids = BTreeSet::new();
    for (i, source) in sources.iter().enumerate() {
        let id = str_at(source, "id").unwrap_or("");
        if id.is_empty() || !source_ids.insert(id.to_owned()) {
            errors.push(format!(
                "source_register[{i}]: missing or duplicate source id"
            ));
        }
    }

    let mut profile_map = BTreeMap::new();
    for (i, profile) in profiles.iter().enumerate() {
        let id = str_at(profile, "profile_id").unwrap_or("");
        if id.is_empty() || profile_map.insert(id.to_owned(), profile).is_some() {
            errors.push(format!("profiles[{i}]: missing or duplicate profile id"));
        }
        let refs = array_at(profile, "source_refs")
            .cloned()
            .unwrap_or_default();
        for reference in refs {
            if !reference
                .as_str()
                .is_some_and(|value| source_ids.contains(value))
            {
                errors.push(format!("profiles[{i}]: unresolved source reference"));
            }
        }
    }

    let mut scenario_ids = BTreeSet::new();
    let mut signal_ids = BTreeSet::new();
    for (i, scenario) in scenarios.iter().enumerate() {
        let scenario_id = str_at(scenario, "scenario_id").unwrap_or("");
        let signal = &scenario["signal"];
        let query = &scenario["query"];
        let signal_id = str_at(signal, "signal_id").unwrap_or("");
        if scenario_id.is_empty() || !scenario_ids.insert(scenario_id.to_owned()) {
            errors.push(format!("scenarios[{i}]: missing or duplicate scenario id"));
        }
        if signal_id.is_empty() || !signal_ids.insert(signal_id.to_owned()) {
            errors.push(format!("scenarios[{i}]: missing or duplicate signal id"));
        }
        let profile_id = str_at(signal, "profile_id").unwrap_or("");
        let Some(profile) = profile_map.get(profile_id) else {
            errors.push(format!("scenarios[{i}]: unresolved profile"));
            continue;
        };
        let assertion_refs = array_at(&signal["assertion"], "source_refs")
            .cloned()
            .unwrap_or_default();
        if assertion_refs.is_empty()
            || assertion_refs.iter().any(|reference| {
                !reference
                    .as_str()
                    .is_some_and(|value| source_ids.contains(value))
            })
        {
            errors.push(format!(
                "scenarios[{i}]: unresolved or missing assertion source references"
            ));
        }
        let (actual_admission, actual_reason) = decision(signal, query, profile);
        let expected_admission = str_at(scenario, "expected_admission").unwrap_or("");
        let expected_reason = str_at(scenario, "expected_reason_code").unwrap_or("");
        let expected_action = str_at(scenario, "expected_action_class").unwrap_or("");
        if actual_admission != expected_admission {
            errors.push(format!(
                "scenarios[{i}]: expected admission {expected_admission}, computed {actual_admission} ({actual_reason})"
            ));
        }
        if actual_reason != expected_reason {
            errors.push(format!(
                "scenarios[{i}]: expected reason {expected_reason}, computed {actual_reason}"
            ));
        }
        let action_class = if actual_admission == "admit_advisory" {
            "advisory_only"
        } else {
            "none"
        };
        if action_class != expected_action {
            errors.push(format!("scenarios[{i}]: unsafe action class promotion"));
        }
    }
    errors
}

fn evaluate(corpus_text: &str, schema_text: &str) -> Result<(Vec<String>, Value, Value), String> {
    let corpus: Value = serde_json::from_str(corpus_text)
        .map_err(|error| format!("corpus JSON parse error: {error}"))?;
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| format!("schema JSON parse error: {error}"))?;
    let mut errors = schema_errors(&schema, &corpus);
    errors.extend(semantic_errors(&corpus));
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
    let mut errors = schema_errors(schema, &changed);
    errors.extend(semantic_errors(&changed));
    if errors.is_empty() {
        return Err(format!("negative control was not rejected: {label}"));
    }
    Ok(())
}

fn run_self_tests(corpus: &Value, schema: &Value) -> Result<usize, String> {
    let baseline_errors = {
        let mut values = schema_errors(schema, corpus);
        values.extend(semantic_errors(corpus));
        values
    };
    if !baseline_errors.is_empty() {
        return Err(format!("baseline corpus failed: {baseline_errors:?}"));
    }

    let mut count = 0;
    mutate_and_reject("invalid signature", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["issuer"]["verification"]["signature_state"] = json!("invalid");
    })?;
    count += 1;

    mutate_and_reject("unknown signing key", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["issuer"]["verification"]["key_state"] = json!("unknown");
    })?;
    count += 1;

    mutate_and_reject("private platform uses section 251", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["sharing"]["lawful_basis_code"] =
            json!("FSR_ACT_251_REGULATOR_SCOPE");
    })?;
    count += 1;

    mutate_and_reject("credit scoring purpose", corpus, schema, |v| {
        v["scenarios"][0]["query"]["requested_purpose"] = json!("credit_scoring");
        v["scenarios"][0]["query"]["requested_action"] = json!("credit_scoring");
    })?;
    count += 1;

    mutate_and_reject("recipient scope widened", corpus, schema, |v| {
        v["scenarios"][0]["query"]["recipient_id"] = json!("BANK-X");
    })?;
    count += 1;

    mutate_and_reject("retention expired", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["sharing"]["retention_expires_at"] =
            json!("2026-11-04T00:00:00Z");
    })?;
    count += 1;

    mutate_and_reject("status contested", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["assertion"]["status"] = json!("disputed");
        v["scenarios"][0]["signal"]["lifecycle"]["correction_status"] = json!("contested");
    })?;
    count += 1;

    mutate_and_reject("public DHT personal data", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["subject"]["data_classification"] = json!("personal");
        v["scenarios"][0]["signal"]["subject"]["subject_type"] = json!("natural_person");
        v["scenarios"][0]["signal"]["sharing"]["storage_scope"] = json!("public_dht");
    })?;
    count += 1;

    mutate_and_reject("global subject token", corpus, schema, |v| {
        v["scenarios"][0]["query"]["recipient_subject_ref"] = json!("global:PERSON-123");
    })?;
    count += 1;

    mutate_and_reject("mirrored evidence counted twice", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["assertion"]["status"] = json!("corroborated");
        v["scenarios"][0]["signal"]["assertion"]["evidence_group_ids"] =
            json!(["SAME-SOURCE", "SAME-SOURCE"]);
    })?;
    count += 1;

    mutate_and_reject(
        "model output promoted to official determination",
        corpus,
        schema,
        |v| {
            v["scenarios"][0]["signal"]["assertion"]["status"] = json!("official_determination");
            v["scenarios"][0]["signal"]["assertion"]["evidence_class"] =
                json!("model_derived_indicator");
            v["scenarios"][0]["signal"]["assertion"]["determining_authority_ref"] = json!("MODEL");
        },
    )?;
    count += 1;

    mutate_and_reject("correction not propagated", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["lifecycle"]["correction_status"] = json!("correction_applied");
        v["scenarios"][0]["signal"]["lifecycle"]["previous_recipient_count"] = json!(4);
        v["scenarios"][0]["signal"]["lifecycle"]["notified_recipient_count"] = json!(1);
    })?;
    count += 1;

    mutate_and_reject("irreversible denial", corpus, schema, |v| {
        v["scenarios"][0]["query"]["requested_action"] = json!("automatic_irreversible_denial");
    })?;
    count += 1;

    mutate_and_reject("payment execution", corpus, schema, |v| {
        v["scenarios"][0]["query"]["requested_action"] = json!("execute_payment");
    })?;
    count += 1;

    mutate_and_reject(
        "special personal information without review",
        corpus,
        schema,
        |v| {
            v["scenarios"][0]["signal"]["subject"]["data_classification"] =
                json!("special_personal_alleged_offence");
            v["scenarios"][0]["signal"]["subject"]["subject_type"] = json!("natural_person");
            v["scenarios"][0]["signal"]["sharing"]["special_info_review_status"] = json!("pending");
            v["scenarios"][0]["signal"]["sharing"]["special_info_review_ref"] = Value::Null;
        },
    )?;
    count += 1;

    mutate_and_reject("high confidence without method", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["assertion"]["confidence_level"] = json!("high");
        v["scenarios"][0]["signal"]["assertion"]["confidence_method_ref"] = Value::Null;
    })?;
    count += 1;

    mutate_and_reject("unknown lawful basis", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["sharing"]["lawful_basis_status"] = json!("unknown");
        v["scenarios"][0]["signal"]["sharing"]["lawful_basis_ref"] = Value::Null;
    })?;
    count += 1;

    mutate_and_reject("external effects enabled", corpus, schema, |v| {
        v["external_effects_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject(
        "pairwise subject token bound to another recipient",
        corpus,
        schema,
        |v| {
            v["scenarios"][0]["query"]["recipient_subject_ref"] =
                json!("pairwise:BANK-C:SUBJECT-001");
        },
    )?;
    count += 1;

    mutate_and_reject("correction receipt identity missing", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["lifecycle"]["correction_status"] = json!("correction_applied");
        v["scenarios"][0]["signal"]["lifecycle"]["previous_recipient_count"] = json!(1);
        v["scenarios"][0]["signal"]["lifecycle"]["previous_recipient_ids"] = json!(["BANK-C"]);
    })?;
    count += 1;

    mutate_and_reject("subject binding unknown", corpus, schema, |v| {
        v["scenarios"][0]["query"]["subject_binding_state"] = json!("unknown");
        v["scenarios"][0]["query"]["subject_binding_receipt_ref"] = Value::Null;
    })?;
    count += 1;

    mutate_and_reject("subject binding mismatch", corpus, schema, |v| {
        v["scenarios"][0]["query"]["subject_binding_state"] = json!("mismatch");
    })?;
    count += 1;

    mutate_and_reject("issuer identity outside profile allowlist", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["issuer"]["participant_id"] = json!("BANK-UNLISTED");
    })?;
    count += 1;

    mutate_and_reject("assertion status outside profile allowlist", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["assertion"]["status"] = json!("official_determination");
    })?;
    count += 1;

    mutate_and_reject("future-dated signal event", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["observed_event"]["observed_at"] =
            json!("2026-11-06T08:00:00Z");
        v["scenarios"][0]["signal"]["observed_event"]["recorded_at"] =
            json!("2026-11-06T08:01:00Z");
    })?;
    count += 1;

    mutate_and_reject("stale signal exceeds profile freshness window", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["observed_event"]["observed_at"] =
            json!("2026-11-02T08:00:00Z");
        v["scenarios"][0]["signal"]["observed_event"]["recorded_at"] =
            json!("2026-11-02T08:01:00Z");
    })?;
    count += 1;

    mutate_and_reject("recorded event precedes observation", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["observed_event"]["recorded_at"] =
            json!("2026-11-04T07:59:00Z");
    })?;
    count += 1;

    mutate_and_reject("appeal closed without decision record", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["lifecycle"]["appeal_status"] = json!("closed");
        v["scenarios"][0]["signal"]["lifecycle"]["appeal_record_ref"] =
            json!("SYN-APPEAL-NEW");
        v["scenarios"][0]["signal"]["lifecycle"]["appeal_decision_ref"] = Value::Null;
    })?;
    count += 1;

    mutate_and_reject("correction notified an unlisted prior recipient", corpus, schema, |v| {
        let lifecycle = &mut v["scenarios"][0]["signal"]["lifecycle"];
        lifecycle["previous_recipient_count"] = json!(1);
        lifecycle["notified_recipient_count"] = json!(1);
        lifecycle["previous_recipient_ids"] = json!(["BANK-C"]);
        lifecycle["notified_recipient_ids"] = json!(["BANK-B"]);
    })?;
    count += 1;

    mutate_and_reject("correction applied without correction record", corpus, schema, |v| {
        let lifecycle = &mut v["scenarios"][0]["signal"]["lifecycle"];
        lifecycle["correction_status"] = json!("correction_applied");
        lifecycle["superseding_signal_id"] = json!("FIS-001-V2");
        lifecycle["correction_record_refs"] = json!([]);
    })?;
    count += 1;

    Ok(count)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut corpus_text = CORPUS_DEFAULT.to_owned();
    let mut schema_text = SCHEMA_DEFAULT.to_owned();
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
                    "sa-payment-fraud-signal-evaluator [--corpus PATH] [--schema PATH] [--self-test]"
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
    let mutation_count = if self_test {
        run_self_tests(&corpus, &schema)?
    } else {
        0
    };
    let status = if errors.is_empty() { "PASS" } else { "FAIL" };
    let report = json!({
        "evaluator": "mycelix.sa-payment-fraud-signal-evaluator.v1",
        "status": status,
        "external_effects_enabled": false,
        "claim_ceiling": "synthetic-only schema and assertion admission evaluation; no production cryptographic verification or legal-basis determination",
        "source_count": array_at(&corpus, "source_register").map_or(0, Vec::len),
        "profile_count": array_at(&corpus, "profiles").map_or(0, Vec::len),
        "scenario_count": array_at(&corpus, "scenarios").map_or(0, Vec::len),
        "negative_controls": mutation_count,
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
    fn checked_in_schema_and_corpus_validate() {
        let (errors, corpus, _) = evaluate(CORPUS_DEFAULT, SCHEMA_DEFAULT).expect("evaluation");
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(array_at(&corpus, "source_register").unwrap().len(), 8);
        assert_eq!(array_at(&corpus, "profiles").unwrap().len(), 6);
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 36);
    }

    #[test]
    fn deterministic_negative_mutations_reject_unsafe_admission() {
        let corpus: Value = serde_json::from_str(CORPUS_DEFAULT).expect("corpus");
        let schema: Value = serde_json::from_str(SCHEMA_DEFAULT).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self tests"), 22);
    }

    #[test]
    fn official_determination_is_not_inferred_from_a_signature() {
        let corpus: Value = serde_json::from_str(CORPUS_DEFAULT).expect("corpus");
        let signal = &corpus["scenarios"][0]["signal"];
        assert_eq!(str_at(&signal["assertion"], "status"), Some("suspected"));
        assert_eq!(
            str_at(&signal["assertion"], "evidence_class"),
            Some("direct_participant_observation")
        );
        assert!(signal["assertion"]["determining_authority_ref"].is_null());
    }
}
