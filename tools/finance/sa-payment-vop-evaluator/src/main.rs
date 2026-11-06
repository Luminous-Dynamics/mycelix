#![forbid(unsafe_code)]

//! Effects-disabled, fail-closed evaluator for Verification of Payee responses.
//! Provider matching decisions remain provider-owned; this evaluator qualifies
//! their type, scope, currentness and the separate payer/authority decision.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::BTreeSet;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-vop-corpus-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-vop-corpus-v1.schema.json");
const EXPECTED_SCHEMA: &str = "mycelix.sa-payment-vop/v1";
const EXPECTED_CORPUS: &str = "ZA-SA-PAYMENT-VOP-SYNTHETIC-V1";

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

fn valid_interval(start: &str, end: &str) -> bool {
    start.len() >= 20
        && end.len() >= 20
        && start.as_bytes().get(10) == Some(&b'T')
        && end.as_bytes().get(10) == Some(&b'T')
        && start < end
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

fn eq_field(left: &Value, left_key: &str, right: &Value, right_key: &str) -> bool {
    str_at(left, left_key) == str_at(right, right_key)
}

fn allowed_outcome(profile: &Value, outcome: &str) -> bool {
    array_at(profile, "allowed_outcomes")
        .is_some_and(|values| values.iter().any(|value| value.as_str() == Some(outcome)))
}

fn is_definitive(outcome: &str) -> bool {
    matches!(outcome, "match" | "close_match" | "no_match")
}

fn qualified_response(
    check: &Value,
    profile: &Value,
    evaluation_at: &str,
    errors: &mut Vec<String>,
    path: &str,
) -> bool {
    let request = &check["request"];
    let response = &check["response"];
    let outcome = str_at(response, "outcome").unwrap_or("unknown");
    let mut bound = true;

    for (request_key, response_key) in [
        ("check_id", "check_id"),
        ("profile_id", "profile_id"),
        ("participant", "participant"),
        (
            "beneficiary_account_commitment",
            "beneficiary_account_commitment",
        ),
        ("alias_state", "alias_state"),
        ("submitted_name_commitment", "submitted_name_commitment"),
        ("matching_profile_version", "matching_profile_version"),
        ("character_set_profile", "character_set_profile"),
        ("payer_audience", "audience"),
        ("request_commitment", "request_commitment"),
        ("payer_subject_commitment", "payer_subject_commitment"),
        (
            "transaction_intent_commitment",
            "transaction_intent_commitment",
        ),
    ] {
        if !eq_field(request, request_key, response, response_key) {
            errors.push(format!(
                "{path}: response field {response_key} is not bound to request field {request_key}"
            ));
            bound = false;
        }
    }

    if request.get("alias_binding_generation") != response.get("alias_binding_generation") {
        errors.push(format!(
            "{path}: alias binding generation differs between request and response"
        ));
        bound = false;
    }

    let requested_at = str_at(request, "requested_at").unwrap_or("");
    let observed_at = str_at(response, "observed_at").unwrap_or("");
    let valid_until = str_at(response, "valid_until").unwrap_or("");
    let fresh = valid_interval(observed_at, valid_until)
        && requested_at <= observed_at
        && observed_at <= evaluation_at
        && evaluation_at < valid_until;
    if !fresh && !matches!(outcome, "stale" | "unknown") {
        errors.push(format!(
            "{path}: non-stale response is outside its request/observation/currentness interval"
        ));
    }

    let signature = str_at(response, "signature_state").unwrap_or("unknown");
    let key_state = str_at(response, "key_state").unwrap_or("unknown");
    let provider_authority = str_at(response, "provider_authority_state").unwrap_or("unknown");
    let audience_matches = eq_field(request, "payer_audience", response, "audience");
    let alias_state = str_at(request, "alias_state").unwrap_or("unknown");
    let alias_generation = request
        .get("alias_binding_generation")
        .and_then(Value::as_u64);

    if alias_state == "current" && alias_generation.is_none_or(|generation| generation == 0) {
        errors.push(format!(
            "{path}: current alias binding requires a positive generation"
        ));
        bound = false;
    }
    if alias_state == "not_applicable" && !request["alias_binding_generation"].is_null() {
        errors.push(format!(
            "{path}: direct-account verification must not invent an alias generation"
        ));
        bound = false;
    }
    if matches!(alias_state, "stale" | "unknown") && outcome == "match" {
        errors.push(format!(
            "{path}: stale or unknown alias binding cannot be promoted to Match"
        ));
        bound = false;
    }

    if !allowed_outcome(profile, outcome) {
        errors.push(format!(
            "{path}: outcome {outcome} is not admitted by source-bound profile"
        ));
        bound = false;
    }
    if str_at(request, "matching_profile_version") != str_at(profile, "matching_profile_version") {
        errors.push(format!(
            "{path}: request matching-profile version differs from the source-bound participant profile"
        ));
        bound = false;
    }
    let charset_allowed =
        array_at(profile, "allowed_character_set_profiles").is_some_and(|values| {
            values
                .iter()
                .any(|value| value.as_str() == str_at(request, "character_set_profile"))
        });
    if !charset_allowed && is_definitive(outcome) {
        errors.push(format!(
            "{path}: definitive matching outcome uses a character-set profile not admitted by the participant profile"
        ));
        bound = false;
    }

    if is_definitive(outcome)
        && (signature != "valid"
            || key_state != "active"
            || provider_authority != "authorised"
            || !audience_matches
            || !fresh
            || !bound)
    {
        errors.push(format!(
            "{path}: definitive VOP outcome lacks valid signature, active key, authorised provider, exact binding and currentness"
        ));
        bound = false;
    }

    if outcome == "match" && str_at(response, "reason_code").is_some() {
        errors.push(format!(
            "{path}: Match cannot carry a failure/uncertainty reason code"
        ));
        bound = false;
    }
    if outcome == "close_match" {
        if str_at(response, "suggested_name_commitment").is_none() {
            errors.push(format!(
                "{path}: Close Match requires a privacy-bounded suggestion commitment"
            ));
            bound = false;
        }
    } else if !response["suggested_name_commitment"].is_null() {
        errors.push(format!(
            "{path}: suggestion may only be present for Close Match"
        ));
        bound = false;
    }

    if outcome == "match" && str_at(profile, "coverage") == Some("unknown") {
        errors.push(format!(
            "{path}: unqualified participant coverage cannot emit an admitted Match"
        ));
        bound = false;
    }

    bound
        && fresh
        && signature == "valid"
        && key_state == "active"
        && provider_authority == "authorised"
        && outcome == "match"
        && alias_state != "stale"
        && alias_state != "unknown"
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
    let sources = array_at(corpus, "source_register")
        .cloned()
        .unwrap_or_default();
    let profiles = array_at(corpus, "profiles").cloned().unwrap_or_default();
    let scenarios = array_at(corpus, "scenarios").cloned().unwrap_or_default();

    check_unique(&sources, "id", "source_register", &mut errors);
    check_unique(&profiles, "profile_id", "profiles", &mut errors);
    check_unique(&scenarios, "scenario_id", "scenarios", &mut errors);

    let source_ids: BTreeSet<String> = sources
        .iter()
        .filter_map(|source| str_at(source, "id").map(str::to_owned))
        .collect();
    let profile_map: std::collections::BTreeMap<String, Value> = profiles
        .iter()
        .filter_map(|profile| {
            str_at(profile, "profile_id").map(|id| (id.to_owned(), profile.clone()))
        })
        .collect();

    for (index, source) in sources.iter().enumerate() {
        if let Some(published) = source.get("published_on").and_then(Value::as_str)
            && !valid_interval(
                &format!("{published}T00:00:00Z"),
                &format!("{}T00:00:00Z", str_at(corpus, "as_of").unwrap_or("")),
            )
        {
            errors.push(format!(
                "source_register[{index}]: publication date is invalid or after as_of"
            ));
        }
    }

    for (index, profile) in profiles.iter().enumerate() {
        let refs = array_at(profile, "source_refs")
            .cloned()
            .unwrap_or_default();
        if refs.is_empty() {
            errors.push(format!("profiles[{index}].source_refs: empty"));
        }
        for reference in refs {
            if !reference.as_str().is_some_and(|id| source_ids.contains(id)) {
                errors.push(format!("profiles[{index}].source_refs: unresolved source"));
            }
        }
        if str_at(profile, "coverage") == Some("unknown")
            && array_at(profile, "allowed_outcomes").is_some_and(|outcomes| {
                outcomes.iter().any(|outcome| {
                    matches!(outcome.as_str(), Some("match" | "close_match" | "no_match"))
                })
            })
        {
            errors.push(format!(
                "profiles[{index}]: unknown coverage may not admit definitive matching outcomes"
            ));
        }
    }

    for (scenario_index, scenario) in scenarios.iter().enumerate() {
        let path = format!("scenarios[{scenario_index}]");
        let checks = array_at(scenario, "checks").cloned().unwrap_or_default();
        let mut check_ids = BTreeSet::new();
        let mut all_checks_qualified = Vec::new();

        for (check_index, check) in checks.iter().enumerate() {
            let check_path = format!("{path}.checks[{check_index}]");
            let request = &check["request"];
            let response = &check["response"];
            let check_id = str_at(request, "check_id").unwrap_or("");
            if !check_ids.insert(check_id.to_owned()) {
                errors.push(format!("{check_path}: duplicate check_id"));
            }

            let profile_id = str_at(request, "profile_id").unwrap_or("");
            let Some(profile) = profile_map.get(profile_id) else {
                errors.push(format!("{check_path}: unresolved profile {profile_id}"));
                all_checks_qualified.push(false);
                continue;
            };
            let qualified =
                qualified_response(check, profile, evaluation_at, &mut errors, &check_path);
            all_checks_qualified.push(qualified);

            let outcome = str_at(response, "outcome").unwrap_or("unknown");
            if outcome == "close_match" {
                if let Some(next) = checks.get(check_index + 1) {
                    if str_at(&next["request"], "submitted_name_commitment")
                        != str_at(response, "suggested_name_commitment")
                    {
                        errors.push(format!(
                            "{check_path}: subsequent check does not use the Close Match suggestion commitment"
                        ));
                    }
                    if str_at(&next["request"], "check_id") == Some(check_id) {
                        errors.push(format!(
                            "{check_path}: Close Match recheck must use a new check identifier"
                        ));
                    }
                }
            }
        }

        let Some(last_check) = checks.last() else {
            errors.push(format!("{path}: at least one check is required"));
            continue;
        };
        let last_request = &last_check["request"];
        let last_response = &last_check["response"];
        let last_outcome = str_at(last_response, "outcome").unwrap_or("unknown");
        let decision = &scenario["payer_decision"];
        let action = str_at(decision, "action").unwrap_or("none");
        let last_check_id = str_at(last_request, "check_id").unwrap_or("");
        let mut accepted_exact = false;

        if action == "accepted_exact" {
            let check_is_qualified = all_checks_qualified.last().copied().unwrap_or(false);
            let decision_matches = str_at(decision, "check_id") == Some(last_check_id)
                && str_at(decision, "confirmed_request_commitment")
                    == str_at(last_request, "request_commitment")
                && str_at(decision, "confirmed_beneficiary_account_commitment")
                    == str_at(last_request, "beneficiary_account_commitment")
                && str_at(decision, "confirmed_name_commitment")
                    == str_at(last_request, "submitted_name_commitment")
                && str_at(decision, "confirmed_payer_subject_commitment")
                    == str_at(last_request, "payer_subject_commitment")
                && str_at(decision, "confirmed_payment_intent_commitment")
                    == str_at(last_request, "transaction_intent_commitment");
            let authority = str_at(decision, "payer_authority_state") == Some("valid");
            let confirmed_at = str_at(decision, "confirmed_at").unwrap_or("");
            let response_observed = str_at(last_response, "observed_at").unwrap_or("");
            let response_expires = str_at(last_response, "valid_until").unwrap_or("");
            let confirmation_current = !confirmed_at.is_empty()
                && response_observed <= confirmed_at
                && confirmed_at <= evaluation_at
                && confirmed_at < response_expires;
            if last_outcome != "match"
                || !check_is_qualified
                || !decision_matches
                || !authority
                || !confirmation_current
            {
                errors.push(format!(
                    "{path}: accepted_exact requires current bound Match, exact payer commitment and separate valid authority"
                ));
            } else {
                accepted_exact = true;
            }
        }

        if action == "reenter_and_recheck" && last_outcome != "close_match" {
            errors.push(format!(
                "{path}: reenter_and_recheck requires a Close Match outcome"
            ));
        }
        if action == "bypass_unverified"
            && !matches!(
                last_outcome,
                "verification_not_possible" | "not_requested" | "unknown" | "stale"
            )
        {
            errors.push(format!(
                "{path}: unverified bypass action conflicts with a definitive provider response"
            ));
        }

        let instruction = &scenario["payment_instruction"];
        if !instruction.is_null() {
            let instruction_matches = accepted_exact
                && str_at(instruction, "source_check_id") == Some(last_check_id)
                && str_at(instruction, "request_commitment")
                    == str_at(last_request, "request_commitment")
                && str_at(instruction, "beneficiary_account_commitment")
                    == str_at(last_request, "beneficiary_account_commitment")
                && str_at(instruction, "confirmed_name_commitment")
                    == str_at(last_request, "submitted_name_commitment")
                && str_at(instruction, "payer_subject_commitment")
                    == str_at(last_request, "payer_subject_commitment")
                && str_at(instruction, "payment_intent_commitment")
                    == str_at(last_request, "transaction_intent_commitment")
                && str_at(instruction, "reference_commitment")
                    == str_at(last_request, "payment_reference_commitment")
                && u64_at(instruction, "amount_minor_units")
                    == u64_at(last_request, "payment_amount_minor_units")
                && str_at(instruction, "currency") == str_at(last_request, "payment_currency")
                && str_at(instruction, "authorization_state") == Some("valid")
                && u64_at(instruction, "amount_minor_units").is_some_and(|amount| amount > 0);
            if !instruction_matches {
                errors.push(format!(
                    "{path}: payment instruction is not bound to a qualified Match and exact payer authority"
                ));
            }
        } else if accepted_exact {
            errors.push(format!(
                "{path}: accepted exact payer decision has no payment instruction result"
            ));
        }

        let expected_effect = if !instruction.is_null() && accepted_exact {
            "payment_instruction_only"
        } else if last_outcome == "close_match" && action == "reenter_and_recheck" {
            "recheck_required"
        } else {
            "none"
        };
        if str_at(scenario, "expected_effect") != Some(expected_effect) {
            errors.push(format!(
                "{path}: expected_effect does not match the qualified check and payer-decision lifecycle"
            ));
        }
        if str_at(scenario, "expected_outcome") != Some(last_outcome) {
            errors.push(format!(
                "{path}: expected_outcome does not match the final provider response"
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
    let mut baseline_errors = semantic_errors(corpus);
    baseline_errors.extend(schema_errors(schema, corpus));
    if !baseline_errors.is_empty() {
        return Err(format!("baseline failed: {baseline_errors:?}"));
    }
    let mut count = 0;

    mutate_and_reject(
        "external effects enabled",
        corpus,
        schema,
        |value| {
            value["external_effects_enabled"] = json!(true);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "invalid signature promoted to Match",
        corpus,
        schema,
        |value| {
            value["scenarios"][18]["checks"][0]["response"]["outcome"] = json!("match");
            value["scenarios"][18]["expected_outcome"] = json!("match");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "revoked key promoted to Match",
        corpus,
        schema,
        |value| {
            value["scenarios"][19]["checks"][0]["response"]["outcome"] = json!("match");
            value["scenarios"][19]["expected_outcome"] = json!("match");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "wrong audience response accepted",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["checks"][0]["response"]["audience"] = json!("payer-psp:other");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "wrong beneficiary account promoted to Match",
        corpus,
        schema,
        |value| {
            value["scenarios"][23]["checks"][0]["response"]["beneficiary_account_commitment"] =
                json!("acct-other");
            value["scenarios"][23]["checks"][0]["response"]["outcome"] = json!("match");
            value["scenarios"][23]["expected_outcome"] = json!("match");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "No Match promoted into payer acceptance",
        corpus,
        schema,
        |value| {
            value["scenarios"][9]["payer_decision"]["action"] = json!("accepted_exact");
            value["scenarios"][9]["payer_decision"]["confirmed_request_commitment"] =
                value["scenarios"][9]["checks"][0]["request"]["request_commitment"].clone();
            value["scenarios"][9]["payer_decision"]["confirmed_beneficiary_account_commitment"] =
                value["scenarios"][9]["checks"][0]["request"]["beneficiary_account_commitment"].clone();
            value["scenarios"][9]["payer_decision"]["confirmed_name_commitment"] =
                value["scenarios"][9]["checks"][0]["request"]["submitted_name_commitment"].clone();
            value["scenarios"][9]["payer_decision"]["payer_authority_state"] = json!("valid");
            value["scenarios"][9]["payer_decision"]["confirmed_at"] = json!("2026-11-04T09:01:00Z");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "Close Match accepted without recheck",
        corpus,
        schema,
        |value| {
            value["scenarios"][7]["payer_decision"]["action"] = json!("accepted_exact");
            value["scenarios"][7]["payer_decision"]["confirmed_request_commitment"] =
                value["scenarios"][7]["checks"][0]["request"]["request_commitment"].clone();
            value["scenarios"][7]["payer_decision"]["confirmed_beneficiary_account_commitment"] =
                value["scenarios"][7]["checks"][0]["request"]["beneficiary_account_commitment"].clone();
            value["scenarios"][7]["payer_decision"]["confirmed_name_commitment"] =
                value["scenarios"][7]["checks"][0]["request"]["submitted_name_commitment"].clone();
            value["scenarios"][7]["payer_decision"]["payer_authority_state"] = json!("valid");
            value["scenarios"][7]["payer_decision"]["confirmed_at"] = json!("2026-11-04T09:01:00Z");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "alias generation mismatch",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["checks"][0]["response"]["alias_binding_generation"] = json!(999);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "expired response promoted to Match",
        corpus,
        schema,
        |value| {
            value["scenarios"][16]["checks"][0]["response"]["outcome"] = json!("match");
            value["scenarios"][16]["expected_outcome"] = json!("match");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "payer consent binds wrong name",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["payer_decision"]["action"] = json!("accepted_exact");
            value["scenarios"][0]["payer_decision"]["confirmed_request_commitment"] =
                value["scenarios"][0]["checks"][0]["request"]["request_commitment"].clone();
            value["scenarios"][0]["payer_decision"]["confirmed_beneficiary_account_commitment"] =
                value["scenarios"][0]["checks"][0]["request"]["beneficiary_account_commitment"].clone();
            value["scenarios"][0]["payer_decision"]["confirmed_name_commitment"] = json!("name-other");
            value["scenarios"][0]["payer_decision"]["payer_authority_state"] = json!("valid");
            value["scenarios"][0]["payer_decision"]["confirmed_at"] = json!("2026-11-04T09:01:00Z");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "unauthorised payer accepted exact match",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["payer_decision"]["action"] = json!("accepted_exact");
            value["scenarios"][0]["payer_decision"]["confirmed_request_commitment"] =
                value["scenarios"][0]["checks"][0]["request"]["request_commitment"].clone();
            value["scenarios"][0]["payer_decision"]["confirmed_beneficiary_account_commitment"] =
                value["scenarios"][0]["checks"][0]["request"]["beneficiary_account_commitment"].clone();
            value["scenarios"][0]["payer_decision"]["confirmed_name_commitment"] =
                value["scenarios"][0]["checks"][0]["request"]["submitted_name_commitment"].clone();
            value["scenarios"][0]["payer_decision"]["payer_authority_state"] = json!("invalid");
            value["scenarios"][0]["payer_decision"]["confirmed_at"] = json!("2026-11-04T09:01:00Z");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "unknown ZA coverage promoted to Match",
        corpus,
        schema,
        |value| {
            value["scenarios"][36]["checks"][0]["response"]["outcome"] = json!("match");
            value["scenarios"][36]["expected_outcome"] = json!("match");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "payer subject replayed across checks",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["checks"][0]["response"]["payer_subject_commitment"] =
                json!("payer-subject-other");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "transaction intent replayed across checks",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["checks"][0]["response"]["transaction_intent_commitment"] =
                json!("intent-other");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "request uses unqualified matching profile version",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["checks"][0]["request"]["matching_profile_version"] =
                json!("old-matching-profile");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "lossy character conversion promoted to Match",
        corpus,
        schema,
        |value| {
            value["scenarios"][39]["checks"][0]["response"]["outcome"] = json!("match");
            value["scenarios"][39]["expected_outcome"] = json!("match");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "payer confirms wrong payment intent",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["payer_decision"]["confirmed_payment_intent_commitment"] =
                json!("intent-other");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "instruction amount differs from confirmed intent",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["payment_instruction"]["amount_minor_units"] = json!(9999);
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
                println!("sa-payment-vop-evaluator [--corpus PATH] [--schema PATH] [--self-test]");
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
        "evaluator": "mycelix.sa-payment-vop-evaluator.v1",
        "status": status,
        "external_effects_enabled": false,
        "claim_ceiling": "synthetic VOP response-binding and payer-intent qualification only",
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
    fn checked_in_vop_schema_and_corpus_validate() {
        let (errors, corpus, _) = evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("evaluate");
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(array_at(&corpus, "source_register").unwrap().len(), 8);
        assert_eq!(array_at(&corpus, "profiles").unwrap().len(), 4);
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 44);
    }

    #[test]
    fn negative_mutation_campaign_rejects_vop_status_and_consent_laundering() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self tests"), 18);
    }
}
