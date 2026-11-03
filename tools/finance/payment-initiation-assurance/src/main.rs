//! Effects-disabled reference evaluator for payment-initiation assurance.
//! This validates synthetic scenarios; it does not parse QR+ payloads, resolve
//! real payees, authenticate users, or call any payment rail.

use jsonschema::draft202012;
use serde_json::{json, Value};
use std::collections::BTreeSet;
use std::error::Error;

const CORPUS: &str = include_str!("../../../../mycelix-finance/docs/payment-initiation-assurance-corpus-v1.json");
const SCHEMA: &str = include_str!("../../../../mycelix-finance/docs/payment-initiation-assurance-v1.schema.json");

fn string_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn bool_at(value: &Value, key: &str) -> Option<bool> {
    value.get(key).and_then(Value::as_bool)
}

fn u64_at(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn expected_for(category: &str) -> Option<&'static str> {
    match category {
        "qr_payload_parsed_only" => Some("parsed_unverified"),
        "verified_payee_presented" => Some("awaiting_payer_authorisation"),
        "payee_substitution" => Some("rejected"),
        "amount_after_confirmation" => Some("rejected"),
        "static_qr_no_amount" => Some("requires_payer_amount"),
        "currency_scale_substitution" => Some("rejected"),
        "expired_dynamic_qr" => Some("rejected"),
        "exact_scan_replay" => Some("idempotent_same_intent"),
        "same_nonce_changed_fingerprint" => Some("conflict"),
        "untrusted_display_name" => Some("awaiting_provider_resolution"),
        "stale_directory_result" => Some("indeterminate_unverified"),
        "invalid_provider_signature" => Some("rejected"),
        "wrong_provider_audience" => Some("rejected"),
        "resolution_rate_limit" => Some("throttled"),
        "user_cancel_before_dispatch" => Some("cancelled_no_effect"),
        "payment_timeout_unknown" => Some("indeterminate_same_operation_id"),
        "provider_accepted_without_settlement" => Some("provider_accepted_not_settled"),
        "standard_published_pilot_unverified" => Some("standard_published_pilot_unverified"),
        "recipient_mismatch_requires_new_consent" => Some("reauthorisation_required"),
        _ => None,
    }
}

fn validate_schema(schema_text: &str, corpus_text: &str) -> Result<Value, Vec<String>> {
    let schema: Value =
        serde_json::from_str(schema_text).map_err(|error| vec![format!("schema parse: {error}")])?;
    let corpus: Value =
        serde_json::from_str(corpus_text).map_err(|error| vec![format!("corpus parse: {error}")])?;
    if !jsonschema::meta::is_valid(&schema) {
        return Err(vec!["schema fails Draft 2020-12 meta-schema".into()]);
    }
    let validator = draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
        .map_err(|error| vec![format!("schema compile: {error}")])?;
    let errors: Vec<String> = validator
        .iter_errors(&corpus)
        .map(|error| format!("{}: {error}", error.instance_path()))
        .collect();
    if errors.is_empty() {
        Ok(corpus)
    } else {
        Err(errors)
    }
}

fn validate_semantics(corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if string_at(corpus, "status") != Some("synthetic_reference_only") {
        errors.push("corpus must remain synthetic_reference_only".into());
    }
    if bool_at(corpus, "external_effects_enabled") != Some(false)
        || bool_at(corpus, "execution_enabled") != Some(false)
    {
        errors.push("external effects and execution must remain disabled".into());
    }
    if corpus["qr_plus_profile"]["state"] != "published_standard"
        || corpus["qr_plus_profile"]["execution_rail"] != "selected_provider_rail"
        || corpus["qr_plus_profile"]["pilot_state"] != "unverified_future_milestone"
    {
        errors.push("QR+ publication, underlying rail execution and pilot status must remain distinct".into());
    }

    let cases = match corpus["cases"].as_array() {
        Some(cases) => cases,
        None => {
            errors.push("cases array missing".into());
            return errors;
        }
    };
    if cases.len() != 19 {
        errors.push(format!("expected 19 frozen cases, found {}", cases.len()));
    }
    let mut ids = BTreeSet::new();
    let mut categories = BTreeSet::new();
    for (index, case) in cases.iter().enumerate() {
        let id = string_at(case, "id").unwrap_or_default();
        let category = string_at(case, "category").unwrap_or_default();
        if id.is_empty() || !ids.insert(id.to_owned()) {
            errors.push(format!("cases[{index}]: missing or duplicate id"));
        }
        if category.is_empty() || !categories.insert(category.to_owned()) {
            errors.push(format!("cases[{index}]: missing or duplicate category"));
        }
        if string_at(case, "expected") != expected_for(category) {
            errors.push(format!("cases[{index}]: category-to-disposition mapping mismatch"));
        }
        if bool_at(case, "dispatch_allowed") != Some(false) {
            errors.push(format!("cases[{index}]: synthetic evaluator must not authorize dispatch"));
        }

        let input = &case["input"];
        if bool_at(input, "dispatch_attempted") == Some(true)
            && bool_at(input, "payer_authorised") != Some(true)
            && category != "payment_timeout_unknown"
        {
            errors.push(format!("cases[{index}]: dispatch attempted without payer authorization"));
        }
        if category == "exact_scan_replay"
            && (bool_at(input, "same_operation_id") != Some(true)
                || bool_at(input, "same_fingerprint") != Some(true)
                || u64_at(input, "second_effect_count") != Some(0))
        {
            errors.push("exact replay must preserve identity/fingerprint and zero second effects".into());
        }
        if category == "same_nonce_changed_fingerprint"
            && (bool_at(input, "same_operation_id") != Some(true)
                || bool_at(input, "same_fingerprint") != Some(false))
        {
            errors.push("changed-fingerprint replay must conflict under the same operation identity".into());
        }
        if category == "payment_timeout_unknown"
            && (bool_at(input, "external_effect_possible") != Some(true)
                || bool_at(input, "original_operation_id_retained") != Some(true)
                || string_at(input, "retry_policy") != Some("query_original_only"))
        {
            errors.push("ambiguous provider result must retain original operation identity and query only".into());
        }
        if category == "provider_accepted_without_settlement"
            && (bool_at(input, "provider_accepted") != Some(true)
                || bool_at(input, "settlement_evidence_present") != Some(false)
                || bool_at(input, "recipient_available_evidence_present") != Some(false))
        {
            errors.push("provider acceptance must not be promoted to settlement or recipient availability".into());
        }
        if category == "standard_published_pilot_unverified"
            && (bool_at(input, "standard_published") != Some(true)
                || bool_at(input, "pilot_commenced") != Some(false)
                || bool_at(input, "merchant_implementation_verified") != Some(false))
        {
            errors.push("published standard cannot prove pilot or merchant implementation".into());
        }
        if category == "recipient_mismatch_requires_new_consent"
            && (bool_at(input, "original_consent_invalidated") != Some(true)
                || bool_at(input, "new_consent_obtained") != Some(false)
                || bool_at(input, "dispatch_attempted") != Some(false))
        {
            errors.push("recipient change must invalidate prior consent and prevent dispatch".into());
        }
        if category == "resolution_rate_limit"
            && (u64_at(input, "resolution_requests_in_window")
                .zip(u64_at(input, "limit"))
                .is_none_or(|(requests, limit)| requests <= limit)
                || bool_at(input, "disclosure_over_limit") != Some(false))
        {
            errors.push("proxy resolution rate-limit negative control is not actually over limit".into());
        }
        if category == "static_qr_no_amount"
            && (bool_at(input, "amount_present") != Some(false)
                || bool_at(input, "payer_entered_amount") != Some(false))
        {
            errors.push("static QR with no amount must not invent one".into());
        }
        if category == "invalid_provider_signature"
            && bool_at(input, "provider_signature_valid") != Some(false)
        {
            errors.push("invalid provider signature test must carry a false verifier result".into());
        }
        if category == "wrong_provider_audience"
            && bool_at(input, "provider_audience_valid") != Some(false)
        {
            errors.push("wrong provider audience must fail".into());
        }
        if case["required_evidence"].as_array().is_none_or(|items| items.is_empty())
            || case["negative_controls"].as_array().is_none_or(|items| items.is_empty())
        {
            errors.push(format!("cases[{index}]: evidence and negative controls cannot be empty"));
        }
    }

    if ids.len() != 19 || categories.len() != 19 {
        errors.push("v1 corpus must contain 19 unique IDs and categories".into());
    }
    errors
}

fn mutation_campaign(corpus: &Value) -> Result<usize, String> {
    if !validate_semantics(corpus).is_empty() {
        return Err("baseline corpus must pass before mutation campaign".into());
    }
    let mutations: Vec<(&str, Box<dyn Fn(&mut Value)>)> = vec![
        ("payee substitution marked parsed-only", Box::new(|v| v["cases"][2]["expected"] = json!("parsed_unverified"))),
        ("amount mutation allowed", Box::new(|v| v["cases"][3]["expected"] = json!("awaiting_payer_authorisation"))),
        ("static QR invents amount", Box::new(|v| v["cases"][4]["input"]["amount_present"] = json!(true))),
        ("currency scale mismatch accepted", Box::new(|v| v["cases"][5]["expected"] = json!("awaiting_payer_authorisation"))),
        ("expired QR accepted", Box::new(|v| v["cases"][6]["expected"] = json!("parsed_unverified"))),
        ("exact replay duplicates effect", Box::new(|v| v["cases"][7]["input"]["second_effect_count"] = json!(1))),
        ("changed fingerprint remains idempotent", Box::new(|v| v["cases"][8]["input"]["same_fingerprint"] = json!(true))),
        ("QR display name treated as authority", Box::new(|v| v["cases"][9]["expected"] = json!("payee_verified"))),
        ("stale directory treated as verified", Box::new(|v| v["cases"][10]["expected"] = json!("awaiting_payer_authorisation"))),
        ("invalid provider signature accepted", Box::new(|v| v["cases"][11]["input"]["provider_signature_valid"] = json!(true))),
        ("wrong audience accepted", Box::new(|v| v["cases"][12]["input"]["provider_audience_valid"] = json!(true))),
        ("enumeration over limit disclosed", Box::new(|v| v["cases"][13]["input"]["disclosure_over_limit"] = json!(true))),
        ("timeout retried with new identity", Box::new(|v| v["cases"][15]["input"]["original_operation_id_retained"] = json!(false))),
        ("accepted promoted to settled", Box::new(|v| v["cases"][16]["input"]["settlement_evidence_present"] = json!(true))),
        ("published standard proves pilot", Box::new(|v| v["cases"][17]["input"]["pilot_commenced"] = json!(true))),
        ("recipient changed with old consent", Box::new(|v| v["cases"][18]["input"]["new_consent_obtained"] = json!(true))),
    ];
    for (name, mutate) in &mutations {
        let mut changed = corpus.clone();
        mutate(&mut changed);
        if validate_semantics(&changed).is_empty() {
            return Err(format!("negative control survived: {name}"));
        }
    }
    Ok(mutations.len())
}

fn main() -> Result<(), Box<dyn Error>> {
    let corpus = validate_schema(SCHEMA, CORPUS)
        .map_err(|errors| std::io::Error::new(std::io::ErrorKind::InvalidData, errors.join("; ")))?;
    let errors = validate_semantics(&corpus);
    let mutations = mutation_campaign(&corpus)
        .map_err(|error| std::io::Error::new(std::io::ErrorKind::InvalidData, error))?;
    let report = json!({
        "evaluator": "mycelix.payment-initiation-assurance.v1",
        "status": if errors.is_empty() {"PASS"} else {"FAIL"},
        "cases": corpus["cases"].as_array().map_or(0, Vec::len),
        "negative_mutations_rejected": mutations,
        "external_effects_enabled": false,
        "claim_ceiling": "synthetic initiation and payee-intent semantic checks only; no real QR+ conformance or payment execution"
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if !errors.is_empty() {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, errors.join("; ")).into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corpus_and_schema_are_valid() {
        let corpus = validate_schema(SCHEMA, CORPUS).expect("schema and corpus");
        assert!(validate_semantics(&corpus).is_empty());
    }

    #[test]
    fn qr_parse_does_not_authorize_dispatch() {
        let corpus: Value = serde_json::from_str(CORPUS).expect("corpus JSON");
        let parsed_only = &corpus["cases"][0];
        assert_eq!(parsed_only["expected"], "parsed_unverified");
        assert_eq!(parsed_only["dispatch_allowed"], false);
    }

    #[test]
    fn exact_retry_and_changed_fingerprint_are_distinct() {
        let corpus: Value = serde_json::from_str(CORPUS).expect("corpus JSON");
        assert_eq!(corpus["cases"][7]["expected"], "idempotent_same_intent");
        assert_eq!(corpus["cases"][8]["expected"], "conflict");
    }

    #[test]
    fn timeout_never_creates_new_operation_identity() {
        let corpus: Value = serde_json::from_str(CORPUS).expect("corpus JSON");
        let case = &corpus["cases"][15];
        assert_eq!(case["expected"], "indeterminate_same_operation_id");
        assert_eq!(case["input"]["retry_policy"], "query_original_only");
    }

    #[test]
    fn mutation_campaign_rejects_all_16_controls() {
        let corpus: Value = serde_json::from_str(CORPUS).expect("corpus JSON");
        assert_eq!(mutation_campaign(&corpus).expect("mutations"), 16);
    }
}
