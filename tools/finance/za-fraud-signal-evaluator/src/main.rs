#![forbid(unsafe_code)]

//! Effects-disabled, purpose-bound fraud-signal exchange reference evaluator.
//! It validates envelopes, not people: a signal is never a guilt or payment decision.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/za-fraud-signal-corpus-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/za-fraud-signal-corpus-v1.schema.json");
const EXPECTED_SCHEMA: &str = "mycelix.za-fraud-signal-exchange/v1";
const EXPECTED_CORPUS: &str = "ZA-FRAUD-SIGNAL-SYNTHETIC-V1";
const ALLOWED_ACTIONS: &[&str] = &[
    "triage",
    "request_more_evidence",
    "human_review",
    "aggregate_analysis",
];
const REQUIRED_PROHIBITIONS: &[&str] = &[
    "global_person_scoring",
    "credit_scoring",
    "automatic_payment_denial",
    "automatic_account_closure",
    "essential_service_denial",
    "unrelated_marketing",
    "bulk_identifier_search",
    "unrestricted_onward_disclosure",
];

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
        errors.push("schema is invalid against JSON Schema meta-schema".into());
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

fn unique_ids(values: &[Value], key: &str, label: &str, errors: &mut Vec<String>) {
    let mut seen = BTreeSet::new();
    for (index, value) in values.iter().enumerate() {
        match str_at(value, key) {
            Some(id) if seen.insert(id.to_owned()) => {}
            Some(id) => errors.push(format!("{label}[{index}]: duplicate {key} {id}")),
            None => errors.push(format!("{label}[{index}]: missing {key}")),
        }
    }
}

fn contains(values: &[Value], needle: &str) -> bool {
    values.iter().any(|value| value.as_str() == Some(needle))
}

fn all_recipients_acknowledged(signal: &Value) -> bool {
    let audience = array_at(signal, "audience").cloned().unwrap_or_default();
    let acknowledgements = array_at(&signal["lifecycle"], "recipient_acknowledgements")
        .cloned()
        .unwrap_or_default();
    !audience.is_empty()
        && audience.iter().all(|recipient| {
            acknowledgements.iter().any(|ack| {
                str_at(ack, "recipient_id") == recipient.as_str()
                    && str_at(ack, "delivery_state") == Some("acknowledged")
                    && str_at(ack, "correction_ack_state") == Some("acknowledged")
            })
        })
}

fn is_before_as_of(expires_at: &str, as_of: &str) -> bool {
    expires_at
        .split('T')
        .next()
        .is_some_and(|date| date < as_of)
}

fn semantic_errors(corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    let as_of = str_at(corpus, "as_of").unwrap_or("");
    if str_at(corpus, "schema") != Some(EXPECTED_SCHEMA)
        || str_at(corpus, "corpus_id") != Some(EXPECTED_CORPUS)
        || str_at(corpus, "status") != Some("synthetic_reference_only")
        || str_at(corpus, "jurisdiction") != Some("ZA")
    {
        errors.push("corpus identity/status/jurisdiction mismatch".into());
    }
    if corpus.get("external_effects_enabled").and_then(Value::as_bool) != Some(false) {
        errors.push("external effects must remain disabled".into());
    }
    if u64_at(corpus, "aggregate_minimum_cell_threshold") != Some(20) {
        errors.push("aggregate threshold must match the frozen reference policy".into());
    }

    let sources = array_at(corpus, "source_register")
        .cloned()
        .unwrap_or_default();
    let scenarios = array_at(corpus, "scenarios")
        .cloned()
        .unwrap_or_default();
    unique_ids(&sources, "id", "source_register", &mut errors);
    unique_ids(&scenarios, "scenario_id", "scenarios", &mut errors);
    let source_ids: BTreeSet<String> = sources
        .iter()
        .filter_map(|item| str_at(item, "id").map(str::to_owned))
        .collect();

    let mut signal_ids = BTreeSet::new();
    let mut token_scopes: BTreeMap<String, (String, String, String)> = BTreeMap::new();

    for (index, scenario) in scenarios.iter().enumerate() {
        let label = format!("scenarios[{index}]");
        let Some(signal) = scenario.get("signal") else {
            errors.push(format!("{label}: signal missing"));
            continue;
        };
        let id = str_at(signal, "signal_id").unwrap_or("<missing>");
        if !signal_ids.insert(id.to_owned()) {
            errors.push(format!("{label}: duplicate signal_id {id}"));
        }
        let plane = str_at(signal, "plane").unwrap_or("");
        let purpose = str_at(signal, "purpose").unwrap_or("");
        let audience = array_at(signal, "audience").cloned().unwrap_or_default();
        let issuer = &signal["issuer"];
        let evidence = &signal["evidence"];
        let authority = &signal["authority"];
        let timestamps = &signal["timestamps"];
        let lifecycle = &signal["lifecycle"];
        let retention = &signal["retention"];
        let expected = str_at(signal, "expected_disposition").unwrap_or("");

        if !source_ids.contains(str_at(evidence, "source_ref").unwrap_or("")) {
            errors.push(format!("{label}/{id}: unresolved source reference"));
        }
        if str_at(evidence, "source_commitment").is_none_or(str::is_empty) {
            errors.push(format!("{label}/{id}: source commitment missing"));
        }
        if str_at(evidence, "origin_verification_state") != Some("verified") {
            errors.push(format!("{label}/{id}: origin evidence is not verified"));
        }
        if str_at(issuer, "signature_verification_receipt").is_none_or(str::is_empty) {
            errors.push(format!("{label}/{id}: signature verification receipt missing"));
        }
        if audience.is_empty() || purpose.is_empty() {
            errors.push(format!("{label}/{id}: audience and purpose are required"));
        }
        if str_at(timestamps, "issued_at").is_none_or(str::is_empty)
            || str_at(timestamps, "expires_at").is_none_or(str::is_empty)
            || str_at(timestamps, "issued_at") >= str_at(timestamps, "expires_at")
        {
            errors.push(format!("{label}/{id}: invalid issue/expiry order"));
        }
        if str_at(retention, "policy_id").is_none_or(str::is_empty) {
            errors.push(format!("{label}/{id}: retention policy is missing"));
        }
        if signal.get("raw_personal_data_present").and_then(Value::as_bool) != Some(false) {
            errors.push(format!("{label}/{id}: raw personal data is prohibited"));
        }
        let prohibited = array_at(signal, "prohibited_uses").cloned().unwrap_or_default();
        for required in REQUIRED_PROHIBITIONS {
            if !contains(&prohibited, required) {
                errors.push(format!("{label}/{id}: missing prohibited use {required}"));
            }
        }

        let authority_state = str_at(authority, "review_state").unwrap_or("unknown");
        let basis_reference = str_at(authority, "basis_reference").unwrap_or("");
        let issuer_state = str_at(issuer, "credential_state").unwrap_or("unknown");
        let role_state = str_at(issuer, "role_authority_state").unwrap_or("unknown");
        let retention_state = str_at(retention, "review_state").unwrap_or("unknown");
        let issuer_role = str_at(issuer, "organisational_role").unwrap_or("");
        let basis_type = str_at(authority, "basis_type").unwrap_or("unknown");

        // FSR Act s251 is not admitted as a blanket statutory basis for a private participant.
        let fsr_251_private_misuse = basis_reference == "FSR_ACT_S251"
            && !matches!(issuer_role, "SARB" | "financial_sector_regulator");
        let mut disposition = if fsr_251_private_misuse
            || authority_state == "denied"
            || authority_state == "expired"
            || role_state == "denied"
            || issuer_state == "expired"
            || issuer_state == "revoked_before_issuance"
        {
            "reject"
        } else if authority_state != "verified"
            || basis_reference.is_empty()
            || retention_state != "verified"
            || role_state != "verified"
            || issuer_state == "unknown"
        {
            "quarantine_unknown_authority"
        } else if issuer_state == "revoked_after_issuance" {
            "quarantine_revoked_key_current_use"
        } else if issuer_state != "verified" {
            "quarantine_unknown_authority"
        } else {
            match plane {
                "strategic_aggregate" => {
                    let aggregate = &signal["aggregate"];
                    let denominator = u64_at(aggregate, "denominator").unwrap_or(0);
                    let count = u64_at(aggregate, "cell_count").unwrap_or(0);
                    let threshold = u64_at(aggregate, "suppression_threshold").unwrap_or(0);
                    let suppressed = aggregate
                        .get("suppression_applied")
                        .and_then(Value::as_bool)
                        == Some(true);
                    let review = str_at(aggregate, "reidentification_review").unwrap_or("unknown");
                    let has_subject = signal
                        .get("subject_reference")
                        .is_some_and(|value| !value.is_null());
                    if has_subject
                        || signal.get("aggregate").is_none_or(Value::is_null)
                        || denominator == 0
                        || threshold < 20
                        || (count < threshold && !suppressed)
                        || review != "low"
                    {
                        "reject"
                    } else {
                        "admit_aggregate_only"
                    }
                }
                "operational_case" => {
                    let subject = &signal["subject_reference"];
                    if subject.is_null()
                        || signal.get("aggregate").is_some_and(|value| !value.is_null())
                        || str_at(signal, "case_reference").is_none_or(str::is_empty)
                    {
                        "reject"
                    } else {
                        let token = str_at(subject, "opaque_token").unwrap_or("");
                        let scope = str_at(subject, "scope_id").unwrap_or("");
                        let audience_id = str_at(subject, "audience_id").unwrap_or("");
                        let purpose_id = str_at(subject, "purpose_id").unwrap_or("");
                        let token_profile = str_at(subject, "tokenisation_profile").unwrap_or("");
                        let scope_tuple = (
                            audience_id.to_owned(),
                            purpose_id.to_owned(),
                            scope.to_owned(),
                        );
                        let token_reused_in_another_scope = token_scopes
                            .get(token)
                            .is_some_and(|existing| existing != &scope_tuple);
                        token_scopes
                            .entry(token.to_owned())
                            .or_insert(scope_tuple);
                        if token.is_empty()
                            || scope.is_empty()
                            || purpose_id != purpose
                            || !audience.iter().any(|value| value.as_str() == Some(audience_id))
                            || !matches!(token_profile, "pairwise_scoped" | "case_scoped")
                            || token_reused_in_another_scope
                        {
                            "reject"
                        } else {
                            let epistemic = str_at(signal, "epistemic_status").unwrap_or("");
                            if epistemic == "adjudicated"
                                && (str_at(evidence, "adjudication_reference").is_none_or(str::is_empty)
                                    || str_at(evidence, "corroboration_state") != Some("adjudicated"))
                            {
                                "reject"
                            } else if is_before_as_of(
                                str_at(timestamps, "expires_at").unwrap_or(""),
                                as_of,
                            ) {
                                "quarantine_expired"
                            } else {
                                match str_at(lifecycle, "state").unwrap_or("") {
                                    "disputed" => "quarantine_disputed",
                                    "expired" => "quarantine_expired",
                                    "withdrawn" => {
                                        if all_recipients_acknowledged(signal) {
                                            "quarantine_withdrawn"
                                        } else {
                                            "quarantine_propagation_incomplete"
                                        }
                                    }
                                    "corrected" => {
                                        if all_recipients_acknowledged(signal) {
                                            "quarantine_corrected"
                                        } else {
                                            "quarantine_propagation_incomplete"
                                        }
                                    }
                                    "asserted" | "corroborated" => {
                                        if matches!(epistemic, "alleged" | "observed" | "inferred") {
                                            "admit_scoped_assertion"
                                        } else {
                                            "reject"
                                        }
                                    }
                                    _ => "reject",
                                }
                            }
                        }
                    }
                }
                _ => "reject",
            }
        };

        if basis_type == "unknown" && disposition == "admit_scoped_assertion" {
            disposition = "quarantine_unknown_authority";
        }

        let permitted = array_at(signal, "permitted_actions").cloned().unwrap_or_default();
        let uses = array_at(signal, "recipient_uses").cloned().unwrap_or_default();
        for (use_index, use_event) in uses.iter().enumerate() {
            let recipient = str_at(use_event, "recipient_id").unwrap_or("");
            let use_purpose = str_at(use_event, "purpose").unwrap_or("");
            let action = str_at(use_event, "action").unwrap_or("");
            if !audience.iter().any(|value| value.as_str() == Some(recipient))
                || use_purpose != purpose
                || !contains(&permitted, action)
                || !ALLOWED_ACTIONS.contains(&action)
                || use_event.get("human_review").and_then(Value::as_bool) != Some(true)
                || str_at(use_event, "separate_decision_reference").is_none_or(str::is_empty)
            {
                errors.push(format!(
                    "{label}/{id}: recipient use {use_index} exceeds purpose/audience/action authority"
                ));
                disposition = "reject";
            }
            if matches!(
                str_at(lifecycle, "state"),
                Some("disputed" | "corrected" | "withdrawn" | "expired")
            ) {
                errors.push(format!(
                    "{label}/{id}: non-current signal cannot support a new recipient use"
                ));
                disposition = "reject";
            }
        }

        if expected != disposition {
            errors.push(format!(
                "{label}/{id}: expected disposition {expected} differs from evaluator {disposition}"
            ));
        }
    }
    errors
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
        return Err(format!("baseline corpus failed: {baseline:?}"));
    }
    let mut count = 0;
    mutate_and_reject("external effects enabled", corpus, schema, |value| {
        value["external_effects_enabled"] = json!(true);
    })?;
    count += 1;
    mutate_and_reject("FSR Act s251 promoted to private blanket basis", corpus, schema, |value| {
        value["scenarios"][4]["signal"]["authority"]["review_state"] = json!("verified");
    })?;
    count += 1;
    mutate_and_reject("small aggregate cell not suppressed", corpus, schema, |value| {
        value["scenarios"][1]["signal"]["aggregate"]["suppression_applied"] = json!(false);
    })?;
    count += 1;
    mutate_and_reject("pairwise token scope widened across audience", corpus, schema, |value| {
        value["scenarios"][9]["signal"]["subject_reference"]["audience_id"] = json!("BANK-C");
    })?;
    count += 1;
    mutate_and_reject("raw personal data in envelope", corpus, schema, |value| {
        value["scenarios"][10]["signal"]["raw_personal_data_present"] = json!(true);
    })?;
    count += 1;
    mutate_and_reject("allegation changed to adjudicated without evidence", corpus, schema, |value| {
        value["scenarios"][7]["signal"]["epistemic_status"] = json!("adjudicated");
    })?;
    count += 1;
    mutate_and_reject("expired signal kept asserted", corpus, schema, |value| {
        value["scenarios"][12]["signal"]["lifecycle"]["state"] = json!("asserted");
    })?;
    count += 1;
    mutate_and_reject("dispute bypassed", corpus, schema, |value| {
        value["scenarios"][13]["signal"]["lifecycle"]["state"] = json!("asserted");
    })?;
    count += 1;
    mutate_and_reject("correction acknowledged before recipient acknowledgement", corpus, schema, |value| {
        value["scenarios"][14]["signal"]["lifecycle"]["recipient_acknowledgements"][0]["correction_ack_state"] = json!("acknowledged");
    })?;
    count += 1;
    mutate_and_reject("recipient purpose changed", corpus, schema, |value| {
        value["scenarios"][17]["signal"]["recipient_uses"][0]["purpose"] = json!("marketing");
    })?;
    count += 1;
    mutate_and_reject("onward disclosure to outside audience", corpus, schema, |value| {
        value["scenarios"][18]["signal"]["recipient_uses"][0]["recipient_id"] = json!("BANK-C");
    })?;
    count += 1;
    mutate_and_reject("automatic payment denial", corpus, schema, |value| {
        value["scenarios"][20]["signal"]["recipient_uses"][0]["action"] = json!("automatic_payment_denial");
    })?;
    count += 1;
    mutate_and_reject("global person score", corpus, schema, |value| {
        value["scenarios"][22]["signal"]["recipient_uses"][0]["action"] = json!("global_person_scoring");
    })?;
    count += 1;
    mutate_and_reject("unknown retention treated as qualified", corpus, schema, |value| {
        value["scenarios"][23]["signal"]["retention"]["review_state"] = json!("verified");
        value["scenarios"][23]["signal"]["retention"]["policy_id"] = json!("RETENTION-CASE-30D");
    })?;
    count += 1;
    mutate_and_reject("revoked key treated as current", corpus, schema, |value| {
        value["scenarios"][25]["signal"]["issuer"]["credential_state"] = json!("verified");
    })?;
    count += 1;
    mutate_and_reject("aggregate plane gains subject reference", corpus, schema, |value| {
        value["scenarios"][0]["signal"]["subject_reference"] = json!({
            "opaque_token":"token",
            "tokenisation_profile":"pairwise_scoped",
            "scope_id":"scope",
            "audience_id":"PEM_STRATEGIC_FORUM",
            "purpose_id":"aggregate_payment_fraud_typology"
        });
    })?;
    count += 1;
    mutate_and_reject("undeclared risk score field", corpus, schema, |value| {
        value["scenarios"][7]["signal"]["risk_score"] = json!(0.01);
    })?;
    count += 1;
    mutate_and_reject("pairwise subject token reused outside original case scope", corpus, schema, |value| {
        value["scenarios"][9]["signal"]["subject_reference"]["opaque_token"] =
            value["scenarios"][8]["signal"]["subject_reference"]["opaque_token"].clone();
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
                println!("za-fraud-signal-evaluator [--corpus PATH] [--schema PATH] [--self-test]");
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
    let corpus: Value = serde_json::from_str(&corpus_text)?;
    let schema: Value = serde_json::from_str(&schema_text)?;
    let mut errors = semantic_errors(&corpus);
    errors.extend(schema_errors(&schema, &corpus));
    let negative_controls = if self_test {
        run_self_tests(&corpus, &schema)?
    } else {
        0
    };
    let report = json!({
        "evaluator":"mycelix.za-fraud-signal-evaluator.v1",
        "status":if errors.is_empty() {"PASS"} else {"FAIL"},
        "external_effects_enabled":false,
        "claim_ceiling":"synthetic signal envelope, purpose, scope, retention and correction validation only",
        "source_count":array_at(&corpus,"source_register").map_or(0,Vec::len),
        "scenario_count":array_at(&corpus,"scenarios").map_or(0,Vec::len),
        "negative_controls":negative_controls,
        "errors":errors
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if !report["status"].as_str().is_some_and(|status| status=="PASS") {
        std::process::exit(1);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn schema_and_synthetic_corpus_validate() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        let mut errors = semantic_errors(&corpus);
        errors.extend(schema_errors(&schema, &corpus));
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(array_at(&corpus, "source_register").unwrap().len(), 7);
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 32);
    }

    #[test]
    fn deterministic_negative_campaign_rejects_seeded_violations() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self tests"), 18);
    }
}
