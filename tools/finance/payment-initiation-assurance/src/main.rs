//! Effects-disabled reference evaluator for payment-initiation assurance.
//! This validates synthetic scenarios; it does not parse QR+ payloads, resolve
//! real payees, authenticate users, or call any payment rail.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::BTreeSet;
use std::error::Error;

const CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/payment-initiation-assurance-corpus-v1.json");
const SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/payment-initiation-assurance-v1.schema.json");

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
        "vop_exact_match" => Some("vop_match_result_bound"),
        "vop_close_match" => Some("vop_close_match_requires_payer_decision"),
        "vop_no_match" => Some("vop_no_match_requires_payer_decision"),
        "vop_unavailable" => Some("vop_unavailable_indeterminate"),
        "vop_other" => Some("vop_other_indeterminate"),
        "vop_stale_result" => Some("vop_stale_result_indeterminate"),
        "vop_wrong_audience" => Some("vop_wrong_audience_rejected"),
        "vop_cross_subject_replay" => Some("vop_cross_subject_replay_rejected"),
        "vop_account_type_mismatch" => Some("vop_account_type_mismatch_disclosed"),
        "vop_match_without_collection_authority" => Some("vop_match_not_collection_authority"),
        "vop_alias_resolved_vop_unavailable" => Some("vop_alias_resolved_vop_unavailable"),
        "vop_subject_changed_after_query" => Some("vop_subject_change_requires_requery"),
        "vop_local_fuzzy_promotion" => Some("vop_local_match_promotion_rejected"),
        "vop_privacy_overdisclosure" => Some("vop_privacy_overdisclosure_rejected"),
        _ => None,
    }
}

fn validate_schema(schema_text: &str, corpus_text: &str) -> Result<Value, Vec<String>> {
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| vec![format!("schema parse: {error}")])?;
    let corpus: Value = serde_json::from_str(corpus_text)
        .map_err(|error| vec![format!("corpus parse: {error}")])?;
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
        errors.push(
            "QR+ publication, underlying rail execution and pilot status must remain distinct"
                .into(),
        );
    }

    let cases = match corpus["cases"].as_array() {
        Some(cases) => cases,
        None => {
            errors.push("cases array missing".into());
            return errors;
        }
    };
    if cases.len() != 33 {
        errors.push(format!("expected 33 frozen cases, found {}", cases.len()));
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
            errors.push(format!(
                "cases[{index}]: category-to-disposition mapping mismatch"
            ));
        }
        if bool_at(case, "dispatch_allowed") != Some(false) {
            errors.push(format!(
                "cases[{index}]: synthetic evaluator must not authorize dispatch"
            ));
        }

        let input = &case["input"];
        if bool_at(input, "dispatch_attempted") == Some(true)
            && bool_at(input, "payer_authorised") != Some(true)
            && category != "payment_timeout_unknown"
        {
            errors.push(format!(
                "cases[{index}]: dispatch attempted without payer authorization"
            ));
        }
        if category == "exact_scan_replay"
            && (bool_at(input, "same_operation_id") != Some(true)
                || bool_at(input, "same_fingerprint") != Some(true)
                || u64_at(input, "second_effect_count") != Some(0))
        {
            errors.push(
                "exact replay must preserve identity/fingerprint and zero second effects".into(),
            );
        }
        if category == "same_nonce_changed_fingerprint"
            && (bool_at(input, "same_operation_id") != Some(true)
                || bool_at(input, "same_fingerprint") != Some(false))
        {
            errors.push(
                "changed-fingerprint replay must conflict under the same operation identity".into(),
            );
        }
        if category == "payment_timeout_unknown"
            && (bool_at(input, "external_effect_possible") != Some(true)
                || bool_at(input, "original_operation_id_retained") != Some(true)
                || string_at(input, "retry_policy") != Some("query_original_only"))
        {
            errors.push(
                "ambiguous provider result must retain original operation identity and query only"
                    .into(),
            );
        }
        if category == "provider_accepted_without_settlement"
            && (bool_at(input, "provider_accepted") != Some(true)
                || bool_at(input, "settlement_evidence_present") != Some(false)
                || bool_at(input, "recipient_available_evidence_present") != Some(false))
        {
            errors.push(
                "provider acceptance must not be promoted to settlement or recipient availability"
                    .into(),
            );
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
            errors
                .push("recipient change must invalidate prior consent and prevent dispatch".into());
        }
        if category == "resolution_rate_limit"
            && (u64_at(input, "resolution_requests_in_window")
                .zip(u64_at(input, "limit"))
                .is_none_or(|(requests, limit)| requests <= limit)
                || bool_at(input, "disclosure_over_limit") != Some(false))
        {
            errors.push(
                "proxy resolution rate-limit negative control is not actually over limit".into(),
            );
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
            errors
                .push("invalid provider signature test must carry a false verifier result".into());
        }
        if category == "wrong_provider_audience"
            && bool_at(input, "provider_audience_valid") != Some(false)
        {
            errors.push("wrong provider audience must fail".into());
        }
        if category.starts_with("vop_") {
            let verification = &case["payee_verification"];
            if !verification.is_object() {
                errors.push(format!(
                    "cases[{index}]: VoP scenario lacks typed payee_verification result"
                ));
            } else {
                let scheme_profile = string_at(verification, "scheme_profile")
                    .unwrap_or("PROVIDER_SPECIFIC_UNKNOWN");
                let result = string_at(verification, "result").unwrap_or("unknown");
                let signature =
                    string_at(verification, "provider_signature_state").unwrap_or("unknown");
                let fraud_inference =
                    string_at(verification, "fraud_inference").unwrap_or("unknown");
                let participant = string_at(verification, "participant").unwrap_or("");
                let issuer = string_at(verification, "issuer").unwrap_or("");
                let audience = string_at(verification, "audience").unwrap_or("");
                let query_subject = string_at(verification, "query_subject_ref").unwrap_or("");
                let response_subject =
                    string_at(verification, "response_subject_ref").unwrap_or("");
                let intent_subject = string_at(verification, "intent_subject_ref").unwrap_or("");
                let requested_type =
                    string_at(verification, "requested_account_type").unwrap_or("not_specified");
                let returned_type =
                    string_at(verification, "returned_account_type").unwrap_or("unknown");
                let current = bool_at(verification, "is_current") == Some(true);
                let audience_matches = bool_at(verification, "audience_matches") == Some(true);
                let payer_notified = bool_at(verification, "payer_notified") == Some(true);
                let payer_decision = string_at(verification, "payer_decision").unwrap_or("none");
                let collection_authority =
                    string_at(verification, "collection_authority_state").unwrap_or("unknown");
                let match_engine = string_at(verification, "match_engine").unwrap_or("unknown");
                let disclosure =
                    string_at(verification, "disclosure_state").unwrap_or("overdisclosed");
                let source_refs = verification["source_refs"].as_array();
                let observed_at = string_at(verification, "observed_at").unwrap_or("");
                let valid_until = string_at(verification, "valid_until").unwrap_or("");

                if participant.is_empty() || issuer.is_empty() || audience.is_empty() {
                    errors.push(format!(
                        "cases[{index}]: VoP result lacks participant, issuer or audience binding"
                    ));
                }
                let proposal_only_unavailable = scheme_profile == "ZA_NPU_PROPOSAL_ONLY"
                    && result == "unavailable"
                    && signature == "unknown"
                    && !audience_matches;
                if signature != "valid" && !proposal_only_unavailable {
                    errors.push(format!(
                        "cases[{index}]: VoP provider response is not authenticated under its declared profile"
                    ));
                }
                if !matches!(fraud_inference, "not_inferred" | "unknown") {
                    errors.push(format!(
                        "cases[{index}]: fraud cannot be inferred from a name-verification result"
                    ));
                }
                if source_refs.is_none_or(|items| items.is_empty()) {
                    errors.push(format!(
                        "cases[{index}]: VoP result lacks source/profile evidence"
                    ));
                }
                let profile_result_compatible = match scheme_profile {
                    "EU_IPR_VOP" => {
                        matches!(result, "match" | "close_match" | "no_match" | "other")
                    }
                    "UK_FPS_COP" => {
                        matches!(result, "match" | "close_match" | "no_match" | "unavailable")
                    }
                    "ZA_NPU_PROPOSAL_ONLY" => {
                        result == "unavailable"
                            && signature == "unknown"
                            && !audience_matches
                            && !current
                    }
                    "PROVIDER_SPECIFIC_UNKNOWN" => {
                        matches!(result, "unknown" | "unavailable" | "other") && !current
                    }
                    _ => false,
                };
                if !profile_result_compatible {
                    errors.push(format!(
                        "cases[{index}]: result class is incompatible with its source scheme profile"
                    ));
                }
                let required_profile_source = match scheme_profile {
                    "EU_IPR_VOP" => Some("ECB_INSTANT_PAYMENTS_REGULATION_2024_886"),
                    "UK_FPS_COP" => Some("PAYUK_FPS_PRINCIPLES_V11_COP_2026"),
                    "ZA_NPU_PROPOSAL_ONLY" => Some("SARB_PEM_POSITION_2025"),
                    _ => None,
                };
                if let Some(required_ref) = required_profile_source
                    && !source_refs.is_some_and(|items| {
                        items.iter().any(|item| item.as_str() == Some(required_ref))
                    })
                {
                    errors.push(format!(
                        "cases[{index}]: scheme profile is missing its authoritative source reference"
                    ));
                }
                let account_type_consistent = match scheme_profile {
                    "EU_IPR_VOP" | "ZA_NPU_PROPOSAL_ONLY" => {
                        requested_type == "not_specified" && returned_type == "unknown"
                    }
                    "UK_FPS_COP" if result == "unavailable" => {
                        requested_type == "not_specified" && returned_type == "unknown"
                    }
                    "UK_FPS_COP" if result == "no_match" => {
                        requested_type == returned_type
                            || (requested_type != "not_specified"
                                && returned_type != "unknown")
                    }
                    "UK_FPS_COP" => {
                        requested_type == returned_type
                            && requested_type != "not_specified"
                            && returned_type != "unknown"
                    }
                    _ => requested_type == "not_specified" && returned_type == "unknown",
                };
                if !account_type_consistent {
                    errors.push(format!(
                        "cases[{index}]: account-type semantics are not qualified by this scheme profile"
                    ));
                }
                if let Some(items) = source_refs {
                    let known_refs: BTreeSet<&str> = corpus["rulebook_references"]
                        .as_array()
                        .into_iter()
                        .flatten()
                        .filter_map(|item| string_at(item, "id"))
                        .collect();
                    for item in items {
                        if item
                            .as_str()
                            .is_none_or(|reference| !known_refs.contains(reference))
                        {
                            errors.push(format!(
                                "cases[{index}]: VoP result contains unresolved source reference"
                            ));
                        }
                    }
                }
                if !observed_at.ends_with('Z')
                    || !valid_until.ends_with('Z')
                    || valid_until <= observed_at
                {
                    errors.push(format!(
                        "cases[{index}]: VoP validity interval is missing or non-positive"
                    ));
                }
                if bool_at(
                    verification,
                    "collection_authority_inferred_from_name_match",
                ) != Some(false)
                {
                    errors.push(format!(
                        "cases[{index}]: account-name match was promoted to collection authority"
                    ));
                }
                if disclosure == "overdisclosed" && category != "vop_privacy_overdisclosure" {
                    errors.push(format!(
                        "cases[{index}]: VoP presentation exceeds permitted disclosure profile"
                    ));
                }
                if bool_at(input, "dispatch_attempted") == Some(true) {
                    errors.push(format!(
                        "cases[{index}]: synthetic VoP qualification cannot authorise payment dispatch"
                    ));
                }

                match category {
                    "vop_exact_match" => {
                        if result != "match"
                            || scheme_profile == "ZA_NPU_PROPOSAL_ONLY"
                            || !audience_matches
                            || !current
                            || query_subject != response_subject
                            || response_subject != intent_subject
                            || !account_type_consistent
                            || match_engine != "provider_authoritative"
                            || !payer_notified
                        {
                            errors.push(format!(
                                "cases[{index}]: exact match must be provider-authoritative, fresh, audience/subject/account-type bound and payer-visible"
                            ));
                        }
                    }
                    "vop_close_match" => {
                        if result != "close_match"
                            || !audience_matches
                            || !current
                            || query_subject != response_subject
                            || !payer_notified
                            || payer_decision == "none"
                        {
                            errors.push(format!(
                                "cases[{index}]: close match requires a current subject-bound result and explicit payer decision"
                            ));
                        }
                    }
                    "vop_no_match" => {
                        if result != "no_match"
                            || !audience_matches
                            || !current
                            || !payer_notified
                            || payer_decision == "none"
                        {
                            errors.push(format!(
                                "cases[{index}]: no match must be surfaced and receive explicit payer decision"
                            ));
                        }
                    }
                    "vop_unavailable" | "vop_other" | "vop_alias_resolved_vop_unavailable" => {
                        let expected_result = if category == "vop_other" {
                            "other"
                        } else {
                            "unavailable"
                        };
                        if result != expected_result
                            || !payer_notified
                            || !matches!(payer_decision, "review" | "cancel")
                        {
                            errors.push(format!(
                                "cases[{index}]: unavailable/other must remain indeterminate and visible, never a match"
                            ));
                        }
                        if category == "vop_alias_resolved_vop_unavailable"
                            && string_at(input, "proxy_resolution_state") != Some("resolved")
                        {
                            errors.push(format!(
                                "cases[{index}]: proxy resolution is not separately established"
                            ));
                        }
                    }
                    "vop_stale_result" => {
                        if result != "match"
                            || current
                            || query_subject != response_subject
                            || response_subject != intent_subject
                            || payer_decision != "review"
                        {
                            errors.push(format!(
                                "cases[{index}]: stale positive result must remain indeterminate"
                            ));
                        }
                    }
                    "vop_wrong_audience" => {
                        if audience_matches {
                            errors.push(format!(
                                "cases[{index}]: wrong-audience response cannot be accepted"
                            ));
                        }
                    }
                    "vop_cross_subject_replay" => {
                        if query_subject == response_subject {
                            errors.push(format!(
                                "cases[{index}]: replay control must bind response to a different account subject"
                            ));
                        }
                    }
                    "vop_account_type_mismatch" => {
                        if requested_type == returned_type
                            || result != "no_match"
                            || !payer_notified
                            || payer_decision == "none"
                        {
                            errors.push(format!(
                                "cases[{index}]: account-type mismatch must remain distinct and visible"
                            ));
                        }
                    }
                    "vop_match_without_collection_authority" => {
                        if result != "match"
                            || collection_authority == "verified"
                            || bool_at(
                                verification,
                                "collection_authority_inferred_from_name_match",
                            ) != Some(false)
                        {
                            errors.push(format!(
                                "cases[{index}]: name match cannot establish requester collection authority"
                            ));
                        }
                    }
                    "vop_subject_changed_after_query" => {
                        if query_subject == intent_subject
                            || response_subject != query_subject
                            || current
                            || payer_decision != "review"
                        {
                            errors.push(format!(
                                "cases[{index}]: changed beneficiary must invalidate the previous VoP result and require re-query"
                            ));
                        }
                    }
                    "vop_local_fuzzy_promotion" => {
                        if result != "close_match"
                            || match_engine != "client_fuzzy"
                            || string_at(verification, "client_asserted_result") != Some("match")
                        {
                            errors.push(format!(
                                "cases[{index}]: client-side fuzzy result must not be promoted to provider-authoritative match"
                            ));
                        }
                    }
                    "vop_privacy_overdisclosure" if disclosure != "overdisclosed" => {
                        errors.push(format!(
                            "cases[{index}]: privacy negative control must contain over-disclosure"
                        ));
                    }
                    _ => {}
                }
            }
        }

        if case["required_evidence"]
            .as_array()
            .is_none_or(|items| items.is_empty())
            || case["negative_controls"]
                .as_array()
                .is_none_or(|items| items.is_empty())
        {
            errors.push(format!(
                "cases[{index}]: evidence and negative controls cannot be empty"
            ));
        }
    }

    if ids.len() != 33 || categories.len() != 33 {
        errors.push("v1 corpus must contain 33 unique IDs and categories".into());
    }
    errors
}

type Mutation = (&'static str, fn(&mut Value));

fn mutation_campaign(corpus: &Value) -> Result<usize, String> {
    let baseline_errors = validate_semantics(corpus);
    if !baseline_errors.is_empty() {
        return Err(format!(
            "baseline corpus must pass before mutation campaign: {baseline_errors:?}"
        ));
    }
    let mutations: [Mutation; 33] = [
        ("payee substitution marked parsed-only", |value| {
            value["cases"][2]["expected"] = json!("parsed_unverified");
        }),
        ("amount mutation allowed", |value| {
            value["cases"][3]["expected"] = json!("awaiting_payer_authorisation");
        }),
        ("static QR invents amount", |value| {
            value["cases"][4]["input"]["amount_present"] = json!(true);
        }),
        ("currency scale mismatch accepted", |value| {
            value["cases"][5]["expected"] = json!("awaiting_payer_authorisation");
        }),
        ("expired QR accepted", |value| {
            value["cases"][6]["expected"] = json!("parsed_unverified");
        }),
        ("exact replay duplicates effect", |value| {
            value["cases"][7]["input"]["second_effect_count"] = json!(1);
        }),
        ("changed fingerprint remains idempotent", |value| {
            value["cases"][8]["input"]["same_fingerprint"] = json!(true);
        }),
        ("QR display name treated as authority", |value| {
            value["cases"][9]["expected"] = json!("payee_verified");
        }),
        ("stale directory treated as verified", |value| {
            value["cases"][10]["expected"] = json!("awaiting_payer_authorisation");
        }),
        ("invalid provider signature accepted", |value| {
            value["cases"][11]["input"]["provider_signature_valid"] = json!(true);
        }),
        ("wrong audience accepted", |value| {
            value["cases"][12]["input"]["provider_audience_valid"] = json!(true);
        }),
        ("enumeration over limit disclosed", |value| {
            value["cases"][13]["input"]["disclosure_over_limit"] = json!(true);
        }),
        ("timeout retried with new identity", |value| {
            value["cases"][15]["input"]["original_operation_id_retained"] = json!(false);
        }),
        ("accepted promoted to settled", |value| {
            value["cases"][16]["input"]["settlement_evidence_present"] = json!(true);
        }),
        ("published standard proves pilot", |value| {
            value["cases"][17]["input"]["pilot_commenced"] = json!(true);
        }),
        ("recipient changed with old consent", |value| {
            value["cases"][18]["input"]["new_consent_obtained"] = json!(true);
        }),
        ("stale VoP result promoted to current match", |value| {
            value["cases"][24]["payee_verification"]["is_current"] = json!(true);
        }),
        ("unavailable result promoted to match", |value| {
            value["cases"][22]["payee_verification"]["result"] = json!("match");
        }),
        ("close match relabelled as exact match", |value| {
            value["cases"][20]["payee_verification"]["result"] = json!("match");
        }),
        ("cross-subject response replay accepted", |value| {
            value["cases"][26]["payee_verification"]["response_subject_ref"] =
                json!("opaque:subject-001");
        }),
        ("wrong audience accepted", |value| {
            value["cases"][25]["payee_verification"]["audience_matches"] = json!(true);
        }),
        ("name match promoted to collection authority", |value| {
            value["cases"][28]["payee_verification"]["collection_authority_inferred_from_name_match"] =
                json!(true);
        }),
        ("account type mismatch hidden", |value| {
            value["cases"][27]["payee_verification"]["returned_account_type"] = json!("individual");
        }),
        ("client fuzzy result promoted to provider result", |value| {
            value["cases"][31]["payee_verification"]["match_engine"] =
                json!("provider_authoritative");
        }),
        ("privacy over-disclosure concealed", |value| {
            value["cases"][32]["payee_verification"]["disclosure_state"] = json!("minimal_result");
        }),
        ("beneficiary changed without re-query", |value| {
            value["cases"][30]["payee_verification"]["intent_subject_ref"] =
                json!("opaque:subject-001");
        }),
        ("name mismatch promoted to fraud verdict", |value| {
            value["cases"][21]["payee_verification"]["fraud_inference"] =
                json!("fraud_inferred_from_name_mismatch");
        }),
        ("unavailable and other result classes collapsed", |value| {
            value["cases"][23]["payee_verification"]["result"] = json!("unavailable");
        }),
        ("UK unavailable outcome assigned EU profile", |value| {
            value["cases"][22]["payee_verification"]["scheme_profile"] = json!("EU_IPR_VOP");
        }),
        ("EU other outcome assigned UK profile", |value| {
            value["cases"][23]["payee_verification"]["scheme_profile"] = json!("UK_FPS_COP");
        }),
        (
            "proposal-only South African profile claims a match",
            |value| {
                value["cases"][29]["payee_verification"]["result"] = json!("match");
            },
        ),
        ("UK profile loses its source binding", |value| {
            value["cases"][20]["payee_verification"]["source_refs"] =
                json!(["ECB_INSTANT_PAYMENTS_REGULATION_2024_886"]);
        }),
        (
            "EU profile invents personal/business account type",
            |value| {
                value["cases"][19]["payee_verification"]["requested_account_type"] =
                    json!("business");
            },
        ),
    ];
    for (name, mutate) in mutations {
        let mut changed = corpus.clone();
        mutate(&mut changed);
        if validate_semantics(&changed).is_empty() {
            return Err(format!("negative control survived: {name}"));
        }
    }
    Ok(mutations.len())
}

fn main() -> Result<(), Box<dyn Error>> {
    let corpus = validate_schema(SCHEMA, CORPUS).map_err(|errors| {
        std::io::Error::new(std::io::ErrorKind::InvalidData, errors.join("; "))
    })?;
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
        assert!(!parsed_only["dispatch_allowed"].as_bool().unwrap_or(true));
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
    fn mutation_campaign_rejects_all_33_controls() {
        let corpus: Value = serde_json::from_str(CORPUS).expect("corpus JSON");
        assert_eq!(mutation_campaign(&corpus).expect("mutations"), 33);
    }

    #[test]
    fn provider_vop_outcomes_remain_distinct_and_non_authoritative() {
        let corpus = validate_schema(SCHEMA, CORPUS).expect("schema and corpus");
        let cases = corpus["cases"].as_array().expect("cases array");
        let exact = &cases[19]["payee_verification"];
        let close = &cases[20]["payee_verification"];
        let unavailable = &cases[22]["payee_verification"];
        let no_match = &cases[21]["payee_verification"];
        let account_type_mismatch = &cases[27]["payee_verification"];
        let proposal_only = &cases[29]["payee_verification"];
        assert_eq!(exact["result"], "match");
        assert_eq!(exact["requested_account_type"], "not_specified");
        assert_eq!(exact["returned_account_type"], "unknown");
        assert_eq!(close["result"], "close_match");
        assert_eq!(close["requested_account_type"], "business");
        assert_eq!(close["returned_account_type"], "business");
        assert_eq!(unavailable["result"], "unavailable");
        assert_eq!(unavailable["scheme_profile"], "UK_FPS_COP");
        assert_eq!(
            account_type_mismatch["requested_account_type"],
            "individual"
        );
        assert_eq!(account_type_mismatch["returned_account_type"], "business");
        assert_eq!(no_match["fraud_inference"], "not_inferred");
        assert_eq!(
            exact["collection_authority_inferred_from_name_match"],
            false
        );
        assert_eq!(proposal_only["scheme_profile"], "ZA_NPU_PROPOSAL_ONLY");
        assert_eq!(proposal_only["provider_signature_state"], "unknown");
        assert_eq!(proposal_only["result"], "unavailable");
        assert!(validate_semantics(&corpus).is_empty());
    }
}
