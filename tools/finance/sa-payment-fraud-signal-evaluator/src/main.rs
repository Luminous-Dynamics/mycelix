#![forbid(unsafe_code)]

//! Effects-disabled evaluator for federated payment fraud signals.
//! Source status, recipient-local decisions, model scores and legal liability
//! are separate evidence dimensions. Relays never acquire source authority.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::BTreeSet;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-fraud-signal-corpus-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-fraud-signal-corpus-v1.schema.json");
const EXPECTED_SCHEMA: &str = "mycelix.sa-payment-fraud-signal/v1";
const EXPECTED_CORPUS: &str = "ZA-SA-PAYMENT-FRAUD-SYNTHETIC-V1";

fn str_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn u64_at(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn array_at<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
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

fn source_refs_resolve(
    item: &Value,
    sources: &BTreeSet<String>,
    path: &str,
    errors: &mut Vec<String>,
) {
    let refs = array_at(item, "source_refs").cloned().unwrap_or_default();
    if refs.is_empty() {
        errors.push(format!("{path}.source_refs: empty"));
    }
    for reference in refs {
        if !reference
            .as_str()
            .is_some_and(|id| sources.contains(id))
        {
            errors.push(format!("{path}.source_refs: unresolved source"));
        }
    }
}

fn distinct_origins(signal: &Value) -> (usize, bool) {
    let contributions = array_at(signal, "evidence_contributions")
        .cloned()
        .unwrap_or_default();
    let mut origins = BTreeSet::new();
    let mut duplicate_claim = false;
    for item in &contributions {
        let origin = str_at(item, "origin_case_commitment").unwrap_or("");
        if !origins.insert(origin.to_owned())
            && item.get("independent_claim").and_then(Value::as_bool) == Some(true)
        {
            duplicate_claim = true;
        }
    }
    (origins.len(), duplicate_claim)
}

fn score_disposition(signal: &Value, evaluation_at: &str) -> Option<&'static str> {
    if signal["model_score"].is_null() {
        return None;
    }
    let score = &signal["model_score"];
    let calibration = str_at(score, "calibration_state").unwrap_or("unknown");
    if calibration == "stale" {
        return Some("quarantine_stale_model_score");
    }
    if calibration != "qualified_synthetic_only"
        || str_at(score, "model_id").is_none_or(str::is_empty)
        || str_at(score, "model_version").is_none_or(str::is_empty)
        || str_at(score, "calibration_id").is_none_or(str::is_empty)
        || str_at(score, "cohort_id").is_none_or(str::is_empty)
        || str_at(score, "evaluation_window_start").is_none_or(str::is_empty)
        || str_at(score, "evaluation_window_end").is_none_or(str::is_empty)
        || score.get("uncertainty").is_none_or(Value::is_null)
    {
        return Some("quarantine_unqualified_score");
    }
    None
}

fn required_disposition(signal: &Value, evaluation_at: &str) -> Option<&'static str> {
    let availability = str_at(signal, "availability_state").unwrap_or("signal_present");
    if matches!(availability, "provider_unavailable" | "no_coverage" | "no_signal_returned") {
        return Some("return_unknown_not_clean");
    }

    let status = str_at(signal, "source_status").unwrap_or("unknown");
    if status == "unknown" {
        return Some("preserve_unknown_status");
    }

    if str_at(signal, "signature_state") != Some("valid")
        || str_at(signal, "key_state") != Some("active")
        || str_at(signal, "source_authority_state") != Some("authorised")
    {
        return Some("quarantine_untrusted_source");
    }

    if str_at(signal, "source_role") != Some("originating_psp")
        && matches!(status, "confirmed" | "disproved" | "withdrawn" | "corrected")
    {
        return Some("reject_unauthorised_source_status");
    }

    let events = array_at(signal, "events").cloned().unwrap_or_default();
    for event in &events {
        let effect = str_at(event, "source_status_effect").unwrap_or("unchanged");
        let actor = str_at(event, "actor_role").unwrap_or("unknown");
        let event_type = str_at(event, "event_type").unwrap_or("");
        if event_type == "recipient_local_decision" && effect != "unchanged" {
            return Some("keep_local_decision_separate");
        }
        if effect != "unchanged"
            && (actor != "originating_psp"
                || str_at(event, "authority_state") != Some("valid"))
        {
            return Some("quarantine_status_laundering");
        }
        if effect == "confirmed"
            && str_at(signal, "evidence_class") == Some("model_score_only")
        {
            return Some("reject_score_as_sole_confirmation");
        }
    }

    let expires_at = str_at(signal, "expires_at").unwrap_or("");
    if !expires_at.is_empty() && expires_at <= evaluation_at {
        return Some("reject_stale_signal");
    }

    let actual_recipient = str_at(signal, "actual_recipient").unwrap_or("");
    let audiences = array_at(signal, "permitted_audiences").cloned().unwrap_or_default();
    let recipient_allowed = audiences
        .iter()
        .any(|audience| audience.as_str() == Some(actual_recipient));
    if !recipient_allowed {
        if events.iter().any(|event| {
            str_at(event, "event_type") == Some("unauthorised_disclosure_attempt_blocked")
        }) {
            return Some("reject_unauthorised_disclosure");
        }
        return Some("reject_wrong_audience");
    }

    let purpose = str_at(signal, "purpose").unwrap_or("unknown");
    let requested = str_at(signal, "requested_purpose").unwrap_or("unknown");
    if purpose == "unknown"
        || requested != purpose
        || matches!(requested, "unrelated_credit_scoring" | "unrelated_disclosure" | "marketing")
    {
        return Some("reject_secondary_purpose");
    }

    let subject = str_at(signal, "subject_reference").unwrap_or("");
    if subject.starts_with("sha256:") || subject.starts_with("phone:") || subject.starts_with("account:") {
        return Some("reject_public_stable_identifier");
    }
    let privacy = array_at(signal, "privacy_findings").cloned().unwrap_or_default();
    if privacy.iter().any(|item| {
        matches!(
            item.as_str(),
            Some("raw_pii_present_in_share_attempt" | "raw_pii_detected_at_ingest_redacted_before_share")
        )
    }) {
        return Some("reject_raw_pii");
    }

    let (distinct_count, duplicate_claim) = distinct_origins(signal);
    let reported_count = u64_at(signal, "independent_evidence_count").unwrap_or(0) as usize;
    if duplicate_claim || reported_count > distinct_count {
        return Some("deduplicate_common_origin_evidence");
    }

    if let Some(score_disposition) = score_disposition(signal, evaluation_at) {
        return Some(score_disposition);
    }
    if str_at(signal, "channel_coverage_state") != Some("qualified_synthetic")
        && !signal["model_score"].is_null()
    {
        return Some("declare_channel_coverage_unknown");
    }

    if status == "confirmed"
        && str_at(signal, "evidence_class") == Some("model_score_only")
    {
        return Some("reject_score_as_sole_confirmation");
    }

    if matches!(status, "disproved" | "withdrawn" | "corrected")
        && str_at(signal, "correction_propagation_state") == Some("pending")
    {
        return Some("retain_correction_pending_state");
    }
    if matches!(status, "disproved" | "withdrawn" | "corrected")
        && str_at(signal, "correction_propagation_state") == Some("complete")
    {
        let recipients = array_at(signal, "correction_recipients")
            .cloned()
            .unwrap_or_default();
        if recipients.iter().any(|recipient| {
            str_at(recipient, "acknowledgement_state") == Some("pending")
        }) {
            return Some("retain_correction_pending_state");
        }
    }
    if matches!(
        str_at(signal, "local_decision"),
        Some("block_local" | "dispute_opened" | "appeal_opened" | "escalate")
    ) && str_at(signal, "local_authority_state") != Some("valid")
    {
        return Some("reject_unauthorised_local_decision");
    }

    match str_at(signal, "evidence_class") {
        Some("payee_name_mismatch_only") => Some("no_fraud_conclusion_from_vop_alone"),
        Some("customer_misrouting_report") => Some("preserve_unadjudicated_case"),
        Some("indicator_only") => Some("keep_indicator_as_suspicion"),
        _ => None,
    }
}

fn semantic_errors(corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
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

    let evaluation_at = str_at(corpus, "evaluation_at").unwrap_or("");
    let sources = array_at(corpus, "source_register").cloned().unwrap_or_default();
    let profiles = array_at(corpus, "profiles").cloned().unwrap_or_default();
    let scenarios = array_at(corpus, "scenarios").cloned().unwrap_or_default();
    check_unique(&sources, "id", "source_register", &mut errors);
    check_unique(&profiles, "profile_id", "profiles", &mut errors);
    check_unique(&scenarios, "scenario_id", "scenarios", &mut errors);

    let source_ids: BTreeSet<String> = sources
        .iter()
        .filter_map(|item| str_at(item, "id").map(str::to_owned))
        .collect();
    let profile_ids: BTreeSet<String> = profiles
        .iter()
        .filter_map(|item| str_at(item, "profile_id").map(str::to_owned))
        .collect();

    for (index, profile) in profiles.iter().enumerate() {
        source_refs_resolve(profile, &source_ids, &format!("profiles[{index}]"), &mut errors);
    }

    for (index, scenario) in scenarios.iter().enumerate() {
        let path = format!("scenarios[{index}]");
        let signal = &scenario["signal"];
        let profile_id = str_at(signal, "profile_id").unwrap_or("");
        if !profile_ids.contains(profile_id) {
            errors.push(format!("{path}: unresolved profile {profile_id}"));
        }

        let events = array_at(signal, "events").cloned().unwrap_or_default();
        let mut event_ids = BTreeSet::new();
        let mut prior_sequence = 0_u64;
        let mut last_source_status: Option<String> = None;
        for (event_index, event) in events.iter().enumerate() {
            let event_path = format!("{path}.signal.events[{event_index}]");
            let event_id = str_at(event, "event_id").unwrap_or("");
            if !event_ids.insert(event_id.to_owned()) {
                errors.push(format!("{event_path}: duplicate event id"));
            }
            let sequence = u64_at(event, "sequence").unwrap_or(0);
            if sequence <= prior_sequence {
                errors.push(format!("{event_path}: sequence must be strictly increasing"));
            }
            prior_sequence = sequence;

            let observed = str_at(event, "observed_at").unwrap_or("");
            let recorded = str_at(event, "record_time").unwrap_or("");
            if observed > recorded {
                errors.push(format!("{event_path}: record_time precedes event time"));
            }

            let effect = str_at(event, "source_status_effect").unwrap_or("unchanged");
            if effect != "unchanged"
                && str_at(event, "actor_role") == Some("originating_psp")
                && str_at(event, "authority_state") == Some("valid")
            {
                last_source_status = Some(effect.to_owned());
            }
        }

        if let Some(last_status) = last_source_status
            && last_status != str_at(signal, "source_status").unwrap_or("")
        {
            errors.push(format!(
                "{path}: envelope source_status differs from latest authorised origin status event"
            ));
        }

        if str_at(signal, "source_status") == Some("confirmed")
            && last_source_status.is_none()
            && str_at(scenario, "expected_disposition")
                != Some("reject_unauthorised_source_status")
        {
            errors.push(format!(
                "{path}: confirmed status lacks an authorised origin-source status event"
            ));
        }

        if let Some(required) = required_disposition(signal, evaluation_at)
            && str_at(scenario, "expected_disposition") != Some(required)
        {
            errors.push(format!(
                "{path}: expected disposition does not match fail-closed signal assessment ({required})"
            ));
        }

        if str_at(signal, "source_status") == Some("corrected")
            && str_at(signal, "correction_propagation_state") == Some("complete")
        {
            let recipients = array_at(signal, "correction_recipients").cloned().unwrap_or_default();
            if recipients.iter().any(|recipient| {
                str_at(recipient, "acknowledgement_state") != Some("acknowledged")
                    || recipient.get("acknowledged_at").is_none_or(Value::is_null)
            }) {
                errors.push(format!(
                    "{path}: complete correction propagation requires an acknowledgement from every recipient"
                ));
            }
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
    let mut baseline_errors = semantic_errors(corpus);
    baseline_errors.extend(schema_errors(schema, corpus));
    if !baseline_errors.is_empty() {
        return Err(format!("baseline failed: {baseline_errors:?}"));
    }
    let mut count = 0;

    mutate_and_reject("external effects enabled", corpus, schema, |value| {
        value["external_effects_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject("relay promotion hidden as harmless forward", corpus, schema, |value| {
        value["scenarios"][2]["signal"]["events"][1]["source_status_effect"] = json!("unchanged");
    })?;
    count += 1;

    mutate_and_reject("confirmed status without authoritative event", corpus, schema, |value| {
        value["scenarios"][1]["signal"]["events"][1]["actor_role"] = json!("relay_participant");
    })?;
    count += 1;

    mutate_and_reject("score-only confirmation treated as valid", corpus, schema, |value| {
        value["scenarios"][16]["signal"]["evidence_class"] =
            json!("independently_verified_case_evidence");
    })?;
    count += 1;

    mutate_and_reject("duplicate origin counted as independent evidence", corpus, schema, |value| {
        value["scenarios"][12]["signal"]["independent_evidence_count"] = json!(1);
        value["scenarios"][12]["signal"]["evidence_contributions"][1]["independent_claim"] =
            json!(false);
    })?;
    count += 1;

    mutate_and_reject("wrong audience disclosed instead of quarantined", corpus, schema, |value| {
        value["scenarios"][11]["signal"]["dissemination_state"] = json!("shared");
    })?;
    count += 1;

    mutate_and_reject("public stable identifier accepted as pairwise subject", corpus, schema, |value| {
        value["scenarios"][27]["signal"]["subject_reference"] =
            json!("pairwise:synthetic-subject");
    })?;
    count += 1;

    mutate_and_reject("raw PII finding removed without correction", corpus, schema, |value| {
        value["scenarios"][28]["signal"]["privacy_findings"] = json!(["none"]);
    })?;
    count += 1;

    mutate_and_reject("unrelated credit scoring purpose shared", corpus, schema, |value| {
        value["scenarios"][29]["signal"]["dissemination_state"] = json!("shared");
    })?;
    count += 1;

    mutate_and_reject("correction marked complete with pending recipient", corpus, schema, |value| {
        value["scenarios"][25]["signal"]["correction_propagation_state"] = json!("complete");
    })?;
    count += 1;

    mutate_and_reject("local recipient decision mutates source status", corpus, schema, |value| {
        value["scenarios"][31]["signal"]["events"][1]["source_status_effect"] = json!("unchanged");
    })?;
    count += 1;

    mutate_and_reject("expired signal reused as current", corpus, schema, |value| {
        value["scenarios"][7]["signal"]["expires_at"] = json!("2026-11-12T09:00:00Z");
    })?;
    count += 1;

    mutate_and_reject("no signal returned reported as clean suspected case", corpus, schema, |value| {
        value["scenarios"][35]["signal"]["availability_state"] = json!("signal_present");
        value["scenarios"][35]["signal"]["source_status"] = json!("suspected");
    })?;
    count += 1;

    mutate_and_reject("uncalibrated model score accepted", corpus, schema, |value| {
        value["scenarios"][14]["signal"]["model_score"]["calibration_state"] =
            json!("qualified_synthetic_only");
    })?;
    count += 1;

    mutate_and_reject("out-of-scope channel score admitted", corpus, schema, |value| {
        value["scenarios"][36]["signal"]["channel_coverage_state"] = json!("qualified_synthetic");
    })?;
    count += 1;

    mutate_and_reject("unauthorised local recipient decision admitted", corpus, schema, |value| {
        value["scenarios"][31]["signal"]["local_authority_state"] = json!("invalid");
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
        "claim_ceiling": "synthetic signal provenance/privacy/correction conformance only",
        "sources": array_at(&corpus, "source_register").map_or(0, Vec::len),
        "profiles": array_at(&corpus, "profiles").map_or(0, Vec::len),
        "scenarios": array_at(&corpus, "scenarios").map_or(0, Vec::len),
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
    fn checked_in_signal_schema_and_corpus_validate() {
        let (errors, corpus, _) = evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("evaluate");
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(array_at(&corpus, "source_register").unwrap().len(), 4);
        assert_eq!(array_at(&corpus, "profiles").unwrap().len(), 3);
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 38);
    }

    #[test]
    fn negative_mutation_campaign_rejects_signal_laundering_and_privacy_failures() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self tests"), 15);
    }
}
