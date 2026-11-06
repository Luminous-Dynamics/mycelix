#![forbid(unsafe_code)]

//! Composes already-qualified payment evidence without allowing one claim to
//! manufacture another. This evaluator does not perform signature verification,
//! canonical request hashing, name matching, fraud adjudication or settlement.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/payment-initiation-composition-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/payment-initiation-composition-v1.schema.json");
const EXPECTED_SCHEMA: &str = "mycelix.payment-initiation-composition/v1";
const EXPECTED_CORPUS: &str = "PAYMENT-INIT-COMPOSE-SYNTHETIC-V1";
const REQUIRED_FIELDS: [&str; 8] = [
    "requester",
    "beneficiary",
    "amount",
    "currency",
    "fee",
    "reference",
    "purpose",
    "expiry",
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

fn valid_utc(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() != 20 || bytes[10] != b'T' || bytes[19] != b'Z' {
        return false;
    }
    for (index, byte) in bytes.iter().enumerate() {
        let separator = matches!(index, 4 | 7 | 10 | 13 | 16 | 19);
        if separator {
            let expected = match index {
                4 | 7 => b'-',
                10 => b'T',
                13 | 16 => b':',
                19 => b'Z',
                _ => b'?',
            };
            if *byte != expected {
                return false;
            }
        } else if !byte.is_ascii_digit() {
            return false;
        }
    }
    let year = value[0..4].parse::<u32>().unwrap_or(0);
    let month = value[5..7].parse::<u32>().unwrap_or(0);
    let day = value[8..10].parse::<u32>().unwrap_or(0);
    let hour = value[11..13].parse::<u32>().unwrap_or(99);
    let minute = value[14..16].parse::<u32>().unwrap_or(99);
    let second = value[17..19].parse::<u32>().unwrap_or(99);
    year >= 1970 && (1..=12).contains(&month) && (1..=31).contains(&day)
        && hour <= 23 && minute <= 59 && second <= 60
}

fn valid_date(value: &str) -> bool {
    value.len() == 10
        && value.as_bytes().get(4) == Some(&b'-')
        && value.as_bytes().get(7) == Some(&b'-')
        && value[0..4].parse::<u32>().is_ok()
        && (1..=12).contains(&value[5..7].parse::<u32>().unwrap_or(0))
        && (1..=31).contains(&value[8..10].parse::<u32>().unwrap_or(0))
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

fn issuer_is_trusted(
    corpus: &Value,
    issuer: &str,
    claim_type: &str,
    audience: &str,
    profile_id: &str,
) -> bool {
    array_at(corpus, "issuer_registry")
        .into_iter()
        .flatten()
        .any(|entry| {
            str_at(entry, "issuer_id") == Some(issuer)
                && array_at(entry, "claim_types")
                    .is_some_and(|items| items.iter().any(|v| v.as_str() == Some(claim_type)))
                && array_at(entry, "audiences")
                    .is_some_and(|items| items.iter().any(|v| v.as_str() == Some(audience)))
                && array_at(entry, "profile_ids")
                    .is_some_and(|items| items.iter().any(|v| v.as_str() == Some(profile_id)))
        })
}

fn profile_by_id<'a>(corpus: &'a Value, profile_id: &str) -> Option<&'a Value> {
    array_at(corpus, "profiles")?
        .iter()
        .find(|profile| str_at(profile, "profile_id") == Some(profile_id))
}

fn validate_common_claim(
    corpus: &Value,
    assertion: &Value,
    claim_type: &str,
    expected_subject: &str,
    profile_id: &str,
    audience: &str,
    evaluation_time: &str,
) -> Result<(), String> {
    if str_at(assertion, "claim_type") != Some(claim_type) {
        return Err(format!("{claim_type}: claim type mismatch"));
    }
    if str_at(assertion, "subject_digest") != Some(expected_subject) {
        return Err(format!("{claim_type}: cross-subject binding mismatch"));
    }
    if str_at(assertion, "profile_id") != Some(profile_id) {
        return Err(format!("{claim_type}: profile binding mismatch"));
    }
    if str_at(assertion, "audience") != Some(audience) {
        return Err(format!("{claim_type}: audience mismatch"));
    }
    let issuer = str_at(assertion, "issuer")
        .ok_or_else(|| format!("{claim_type}: issuer missing"))?;
    if !issuer_is_trusted(corpus, issuer, claim_type, audience, profile_id) {
        return Err(format!("{claim_type}: issuer not registered for claim, audience and profile"));
    }
    let observed = str_at(assertion, "observed_at")
        .ok_or_else(|| format!("{claim_type}: observed_at missing"))?;
    let valid_until = str_at(assertion, "valid_until")
        .ok_or_else(|| format!("{claim_type}: valid_until missing"))?;
    if !valid_utc(observed) || !valid_utc(valid_until) || observed > evaluation_time {
        return Err(format!("{claim_type}: invalid or future-dated assertion time"));
    }
    if valid_until < observed || valid_until < evaluation_time {
        return Err(format!("{claim_type}: assertion stale at evaluation time"));
    }
    Ok(())
}

fn push_error(errors: &mut Vec<String>, error: Result<(), String>) {
    if let Err(message) = error {
        errors.push(message);
    }
}

fn validate_scenario(corpus: &Value, scenario: &Value) -> (String, String, Vec<String>, Value) {
    let mut errors = Vec::new();
    let intent = &scenario["intent"];
    let assertions = &scenario["assertions"];
    let payer = &scenario["payer_decision"];
    let scenario_id = str_at(scenario, "scenario_id").unwrap_or("<missing>");
    let now = str_at(scenario, "evaluation_time").unwrap_or("");
    let profile_id = str_at(intent, "profile_id").unwrap_or("");
    let Some(profile) = profile_by_id(corpus, profile_id) else {
        errors.push(format!("{scenario_id}: unknown payment profile"));
        return ("quarantine".into(), "none".into(), errors, json!({}));
    };
    let audience = str_at(profile, "audience").unwrap_or("");
    let request_digest = str_at(intent, "request_digest").unwrap_or("");
    let recomputed_digest = str_at(intent, "recomputed_request_digest").unwrap_or("");
    let payer_subject = str_at(intent, "payer_subject_digest").unwrap_or("");
    let requester_subject = str_at(intent, "requester_subject_digest").unwrap_or("");
    let beneficiary_subject = str_at(intent, "beneficiary_subject_digest").unwrap_or("");
    let beneficiary_binding = str_at(intent, "beneficiary_binding_digest").unwrap_or("");
    let collection_digest = str_at(intent, "collection_authority_digest").unwrap_or("");
    let invoice_digest = str_at(intent, "invoice_digest").unwrap_or("");
    let amount = u64_at(intent, "amount_minor_units").unwrap_or(0);
    let currency = str_at(intent, "currency").unwrap_or("");
    let product = str_at(intent, "product").unwrap_or("");
    let channel = str_at(intent, "channel").unwrap_or("");
    let expiry = str_at(intent, "expiry_at").unwrap_or("");

    if !valid_utc(now) || !valid_utc(expiry) || !valid_utc(str_at(intent, "issued_at").unwrap_or("")) {
        errors.push(format!("{scenario_id}: invalid evaluation/intent timestamp"));
    }
    if request_digest.is_empty() || request_digest != recomputed_digest
        || str_at(intent, "digest_verification_status") != Some("verified")
    {
        errors.push(format!("{scenario_id}: canonical request digest not independently verified"));
    }
    if amount == 0 || currency.len() != 3 || product.is_empty() || channel.is_empty() {
        errors.push(format!("{scenario_id}: incomplete economic intent"));
    }

    // Message authenticity and requester identity are separate assertions.
    let message = &assertions["message_authenticity"];
    let message_state = str_at(message, "state").unwrap_or("unknown");
    if message_state != "verified" {
        errors.push(format!("{scenario_id}: message authenticity is not verified"));
        if message_state == "invalid" {
            errors.push(format!("{scenario_id}: explicit invalid message signature"));
        }
    }
    push_error(
        &mut errors,
        validate_common_claim(
            corpus, message, "message_authenticity", request_digest, profile_id, audience, now,
        ),
    );
    let requester = &assertions["requester_identity"];
    let requester_state = str_at(requester, "state").unwrap_or("unknown");
    if requester_state != "verified" {
        errors.push(format!("{scenario_id}: requester identity is not verified"));
        if requester_state == "invalid" {
            errors.push(format!("{scenario_id}: explicit invalid requester identity"));
        }
    }
    push_error(
        &mut errors,
        validate_common_claim(
            corpus, requester, "requester_identity", requester_subject, profile_id, audience, now,
        ),
    );
    if str_at(requester, "scope") != Some("request:create") {
        errors.push(format!("{scenario_id}: requester identity lacks request:create scope"));
    }

    // VoP/CoP confirms a bounded account/name match; it never establishes legal identity.
    let beneficiary = &assertions["beneficiary_match"];
    let beneficiary_state = str_at(beneficiary, "state").unwrap_or("unknown");
    if !matches!(beneficiary_state, "match" | "close_match") {
        errors.push(format!("{scenario_id}: beneficiary match not sufficient for initiation"));
        if beneficiary_state == "no_match" {
            errors.push(format!("{scenario_id}: explicit no-match payee result"));
        }
    }
    push_error(
        &mut errors,
        validate_common_claim(
            corpus, beneficiary, "beneficiary_match", beneficiary_binding, profile_id, audience, now,
        ),
    );
    if beneficiary_state == "close_match"
        && !array_at(payer, "acknowledged_warnings").is_some_and(|items| {
            items.iter().any(|v| v.as_str() == Some("beneficiary_close_match"))
        })
    {
        errors.push(format!("{scenario_id}: close-match warning not acknowledged by payer"));
    }

    // Collection authority is bound to requester, beneficiary, product and channel.
    let authority = &assertions["collection_authority"];
    if str_at(authority, "state") != Some("verified") {
        errors.push(format!("{scenario_id}: collection authority is not verified"));
        if str_at(authority, "state") == Some("invalid") {
            errors.push(format!("{scenario_id}: explicit invalid collection authority"));
        }
    }
    push_error(
        &mut errors,
        validate_common_claim(
            corpus, authority, "collection_authority", collection_digest, profile_id, audience, now,
        ),
    );
    if str_at(authority, "requester_subject_digest") != Some(requester_subject)
        || str_at(authority, "beneficiary_subject_digest") != Some(beneficiary_subject)
        || str_at(authority, "product") != Some(product)
        || str_at(authority, "channel") != Some(channel)
    {
        errors.push(format!("{scenario_id}: collection authority is not bound to this requester, beneficiary, product and channel"));
    }

    // Commercial evidence is required only by the selected profile/use-case policy.
    let commercial = &assertions["commercial_evidence"];
    let commercial_state = str_at(commercial, "state").unwrap_or("unknown");
    let commercial_required = intent["commercial_evidence_required"].as_bool().unwrap_or(true);
    if commercial_required {
        if commercial_state != "authenticated" {
            errors.push(format!("{scenario_id}: required commercial evidence is not authenticated"));
        }
        push_error(
            &mut errors,
            validate_common_claim(
                corpus, commercial, "commercial_evidence", invoice_digest, profile_id, audience, now,
            ),
        );
        if str_at(commercial, "payer_subject_digest") != Some(payer_subject)
            || u64_at(commercial, "amount_minor_units") != Some(amount)
            || str_at(commercial, "currency") != Some(currency)
        {
            errors.push(format!("{scenario_id}: commercial evidence is not bound to the exact payer, amount and currency"));
        }
    } else if commercial_state != "not_required" {
        errors.push(format!("{scenario_id}: optional commercial evidence must be explicitly marked not_required"));
    }

    // Fraud intelligence remains advisory. Local policy, not a signal, controls disposition.
    let fraud = &assertions["fraud_signal"];
    let fraud_state = str_at(fraud, "state").unwrap_or("unknown");
    push_error(
        &mut errors,
        validate_common_claim(
            corpus, fraud, "fraud_signal", payer_subject, profile_id, audience, now,
        ),
    );
    if str_at(fraud, "purpose") != Some("payment_initiation_abuse_advisory") {
        errors.push(format!("{scenario_id}: fraud assertion has wrong purpose"));
    }

    let local = &assertions["local_policy_decision"];
    let local_state = str_at(local, "state").unwrap_or("unknown");
    let local_valid = str_at(local, "authority_state") == Some("valid");
    let local_binding = validate_common_claim(
        corpus, local, "local_policy_decision", request_digest, profile_id, audience, now,
    );
    if local_binding.is_err() || !local_valid {
        errors.push(format!("{scenario_id}: local policy decision is not authoritative for this request"));
    }

    // Payer decision is a separate authenticated act over the exact displayed request.
    let payer_decision = str_at(payer, "decision").unwrap_or("none");
    let payer_time = str_at(payer, "observed_at").unwrap_or("");
    let payer_valid_until = str_at(payer, "valid_until").unwrap_or("");
    let payer_issuer = str_at(payer, "issuer").unwrap_or("");
    let payer_trusted = issuer_is_trusted(
        corpus, payer_issuer, "payer_decision", audience, profile_id,
    );
    let payer_acceptance_valid =
        str_at(payer, "subject_digest") == Some(request_digest)
            && str_at(payer, "accepted_request_digest") == Some(request_digest)
            && str_at(payer, "payer_subject_digest") == Some(payer_subject)
            && str_at(payer, "audience") == Some(audience)
            && str_at(payer, "profile_id") == Some(profile_id)
            && str_at(payer, "authority_state") == Some("valid")
            && payer_trusted
            && valid_utc(payer_time)
            && valid_utc(payer_valid_until)
            && payer_time <= expiry
            && payer_time <= now
            && payer_valid_until >= now
            && REQUIRED_FIELDS.iter().all(|field| {
                array_at(payer, "acknowledged_fields")
                    .is_some_and(|items| items.iter().any(|v| v.as_str() == Some(field)))
            });

    let mut disposition = if errors.is_empty() {
        "permit_instruction"
    } else if errors.iter().any(|error| {
        error.contains("invalid") || error.contains("cross-subject")
            || error.contains("mismatch") || error.contains("no-match")
            || error.contains("different") || error.contains("not bound")
            || error.contains("digest") || error.contains("not trusted")
            || error.contains("issuer not registered")
            || error.contains("explicit invalid") || error.contains("no-match")
            || error.contains("unsupported") || error.contains("wrong")
    }) {
        "reject"
    } else {
        "quarantine"
    };

    if matches!(payer_decision, "decline" | "query" | "pay_later" | "none") && errors.is_empty() {
        disposition = "no_instruction";
    } else if payer_decision == "accept" && !payer_acceptance_valid {
        errors.push(format!("{scenario_id}: payer acceptance is not valid, exact, current and complete"));
        disposition = "reject";
    } else if payer_decision == "accept" && local_state == "deny" {
        disposition = "reject";
    } else if payer_decision == "accept" && local_state == "review" {
        disposition = "quarantine";
    } else if payer_decision == "accept" && local_state != "allow" {
        disposition = "quarantine";
    }

    let assurance_vector = json!({
        "message_authenticity": message_state,
        "requester_identity": requester_state,
        "beneficiary_match": beneficiary_state,
        "collection_authority": str_at(authority, "state").unwrap_or("unknown"),
        "commercial_evidence": commercial_state,
        "payer_decision": payer_decision,
        "fraud_signal": fraud_state,
        "local_policy_decision": local_state,
        "global_trust_score": Value::Null
    });

    let mut effect_state = "none";
    let mut instruction_id: Option<String> = None;
    let mut previous_sequence = 0_u64;
    let mut previous_time = "";
    let mut seen_events = BTreeSet::new();
    for (index, event) in array_at(scenario, "events").into_iter().flatten().enumerate() {
        let event_id = str_at(event, "event_id").unwrap_or("");
        let sequence = u64_at(event, "sequence").unwrap_or(0);
        let event_type = str_at(event, "event_type").unwrap_or("");
        let actor = str_at(event, "actor").unwrap_or("");
        let authority_state = str_at(event, "authority_state").unwrap_or("");
        let event_subject = str_at(event, "subject_digest").unwrap_or("");
        let event_time = str_at(event, "observed_at").unwrap_or("");
        let event_instruction = str_at(event, "instruction_id");
        let evidence = str_at(event, "evidence_type").unwrap_or("");
        if event_id.is_empty() || !seen_events.insert(event_id.to_owned()) {
            errors.push(format!("{scenario_id}: event[{index}] has missing/duplicate identity"));
        }
        if sequence <= previous_sequence || !valid_utc(event_time) || event_time < previous_time {
            errors.push(format!("{scenario_id}: event sequence/time is invalid"));
        }
        previous_sequence = sequence;
        previous_time = event_time;
        if event_subject != request_digest {
            errors.push(format!("{scenario_id}: event subject does not bind to request digest"));
        }
        match event_type {
            "instruction_creation_attempt_rejected" | "recipient_availability_claim_rejected" => {
                if authority_state != "invalid" || !evidence.ends_with("rejection_receipt") {
                    errors.push(format!("{scenario_id}: rejected attempt must remain a non-effect diagnostic"));
                }
            }
            "instruction_created" => {
                if disposition != "permit_instruction" || actor != "SYNTH-PSP-A"
                    || authority_state != "valid" || evidence != "payment_instruction_receipt"
                {
                    errors.push(format!("{scenario_id}: instruction event lacks exact preconditions or authorised evidence"));
                }
                if instruction_id.is_some() {
                    errors.push(format!("{scenario_id}: multiple instructions for one request"));
                }
                match event_instruction {
                    Some(id) if !id.is_empty() => instruction_id = Some(id.to_owned()),
                    _ => errors.push(format!("{scenario_id}: instruction id missing")),
                }
                effect_state = "instruction_created";
            }
            "provider_outcome_unknown" => {
                if instruction_id.is_none() || event_instruction != instruction_id.as_deref()
                    || actor != "SYNTH-PSP-A" || authority_state != "unknown"
                    || evidence != "provider_timeout_receipt"
                {
                    errors.push(format!("{scenario_id}: provider unknown outcome lacks matching instruction evidence"));
                }
                effect_state = "provider_outcome_unknown";
            }
            "recipient_available" => {
                if instruction_id.is_none() || event_instruction != instruction_id.as_deref()
                    || actor != "SYNTH-PSP-B" || authority_state != "valid"
                    || evidence != "recipient_availability_receipt"
                {
                    errors.push(format!("{scenario_id}: recipient availability lacks matching instruction and beneficiary PSP evidence"));
                }
                effect_state = "recipient_available";
            }
            "settlement_observed" => {
                if effect_state != "recipient_available"
                    || event_instruction != instruction_id.as_deref()
                    || actor != "SYNTH-SETTLEMENT-AUTHORITY"
                    || authority_state != "valid" || evidence != "settlement_statement_record"
                {
                    errors.push(format!("{scenario_id}: settlement observation lacks distinct prior availability and settlement evidence"));
                }
                effect_state = "settlement_observed";
            }
            "finality_qualified" => {
                if effect_state != "settlement_observed"
                    || event_instruction != instruction_id.as_deref()
                    || authority_state != "valid" || evidence != "profile_finality_certificate"
                {
                    errors.push(format!("{scenario_id}: finality lacks separate profile-specific evidence after settlement"));
                }
                effect_state = "finality_qualified";
            }
            _ => errors.push(format!("{scenario_id}: unsupported payment-effect event")),
        }
    }

    let expected_effect = str_at(scenario, "expected_effect_state").unwrap_or("none");
    if effect_state != expected_effect {
        errors.push(format!(
            "{scenario_id}: observed effect state {effect_state} differs from expected {expected_effect}"
        ));
    }
    if instruction_id.is_some() && disposition != "permit_instruction" {
        errors.push(format!("{scenario_id}: instruction exists when composition disposition is {disposition}"));
    }
    if instruction_id.is_none() && matches!(effect_state, "provider_outcome_unknown" | "recipient_available" | "settlement_observed" | "finality_qualified") {
        errors.push(format!("{scenario_id}: later payment-effect state lacks an instruction"));
    }
    let expected_disposition = str_at(scenario, "expected_disposition").unwrap_or("");
    if disposition != expected_disposition {
        errors.push(format!(
            "{scenario_id}: computed disposition {disposition} differs from expected {expected_disposition}"
        ));
    }
    (disposition.into(), effect_state.into(), errors, assurance_vector)
}

fn semantic_errors(corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if str_at(corpus, "schema") != Some(EXPECTED_SCHEMA)
        || str_at(corpus, "corpus_id") != Some(EXPECTED_CORPUS)
        || str_at(corpus, "status") != Some("synthetic_reference_only")
        || corpus.get("external_effects_enabled").and_then(Value::as_bool) != Some(false)
    {
        errors.push("corpus identity/status/effects contract invalid".into());
    }
    let as_of = str_at(corpus, "as_of").unwrap_or("");
    if !valid_date(as_of) {
        errors.push("corpus as_of date invalid".into());
    }

    let sources = array_at(corpus, "source_register").cloned().unwrap_or_default();
    let profiles = array_at(corpus, "profiles").cloned().unwrap_or_default();
    let scenarios = array_at(corpus, "scenarios").cloned().unwrap_or_default();
    let mut source_ids = BTreeSet::new();
    let mut profile_ids = BTreeSet::new();
    let mut issuer_ids = BTreeSet::new();
    let mut scenario_ids = BTreeSet::new();

    for (index, source) in sources.iter().enumerate() {
        let id = str_at(source, "id").unwrap_or("");
        if id.is_empty() || !source_ids.insert(id.to_owned()) {
            errors.push(format!("source_register[{index}]: missing or duplicate id"));
        }
        let observed = str_at(source, "observed_on").unwrap_or("");
        if !valid_date(observed) || observed > as_of {
            errors.push(format!("source_register[{index}]: invalid/future observed date"));
        }
    }
    for (index, profile) in profiles.iter().enumerate() {
        let id = str_at(profile, "profile_id").unwrap_or("");
        if id.is_empty() || !profile_ids.insert(id.to_owned()) {
            errors.push(format!("profiles[{index}]: missing or duplicate profile id"));
        }
        for reference in array_at(profile, "source_refs").into_iter().flatten() {
            if !reference.as_str().is_some_and(|v| source_ids.contains(v)) {
                errors.push(format!("profiles[{index}]: unresolved source reference"));
            }
        }
        let observed = str_at(profile, "observed_on").unwrap_or("");
        if !valid_date(observed) || observed > as_of {
            errors.push(format!("profiles[{index}]: invalid/future observed date"));
        }
    }
    for (index, issuer) in array_at(corpus, "issuer_registry").into_iter().flatten().enumerate() {
        let id = str_at(issuer, "issuer_id").unwrap_or("");
        if id.is_empty() || !issuer_ids.insert(id.to_owned()) {
            errors.push(format!("issuer_registry[{index}]: missing or duplicate issuer id"));
        }
    }
    for (index, scenario) in scenarios.iter().enumerate() {
        let id = str_at(scenario, "scenario_id").unwrap_or("");
        if id.is_empty() || !scenario_ids.insert(id.to_owned()) {
            errors.push(format!("scenarios[{index}]: missing or duplicate scenario id"));
        }
        let (disposition, effect, scenario_errors, _) = validate_scenario(corpus, scenario);
        errors.extend(scenario_errors);
        if disposition.is_empty() || effect.is_empty() {
            errors.push(format!("scenarios[{index}]: evaluation did not return a disposition/effect"));
        }
    }
    if sources.is_empty() || profiles.is_empty() || scenarios.len() < 30 {
        errors.push("corpus lacks required sources, profiles or 30 scenarios".into());
    }
    errors
}

fn mutate_and_expect_rejection(
    label: &str,
    corpus: &Value,
    mutation: impl FnOnce(&mut Value),
) -> Result<(), String> {
    let mut changed = corpus.clone();
    mutation(&mut changed);
    let errors = semantic_errors(&changed);
    if errors.is_empty() {
        return Err(format!("negative mutation was not rejected: {label}"));
    }
    Ok(())
}

fn run_self_tests(corpus: &Value) -> Result<usize, String> {
    if !semantic_errors(corpus).is_empty() {
        return Err("baseline corpus must pass before mutations".into());
    }
    let mut count = 0;
    let mutations: Vec<(&str, Box<dyn FnOnce(&mut Value)>)> = vec![
        ("JWS subject mismatch", Box::new(|v| v["scenarios"][0]["assertions"]["message_authenticity"]["subject_digest"] = json!("sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"))),
        ("JWS wrong audience", Box::new(|v| v["scenarios"][0]["assertions"]["message_authenticity"]["audience"] = json!("OTHER-PSP"))),
        ("JWS untrusted issuer", Box::new(|v| v["scenarios"][0]["assertions"]["message_authenticity"]["issuer"] = json!("UNLISTED"))),
        ("requester cross-subject", Box::new(|v| v["scenarios"][0]["assertions"]["requester_identity"]["subject_digest"] = json!("sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"))),
        ("VoP cross-subject", Box::new(|v| v["scenarios"][0]["assertions"]["beneficiary_match"]["subject_digest"] = json!("sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"))),
        ("collection authority beneficiary mismatch", Box::new(|v| v["scenarios"][0]["assertions"]["collection_authority"]["beneficiary_subject_digest"] = json!("sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"))),
        ("commercial amount mismatch", Box::new(|v| v["scenarios"][0]["assertions"]["commercial_evidence"]["amount_minor_units"] = json!(1))),
        ("payer accepted old digest", Box::new(|v| v["scenarios"][0]["payer_decision"]["accepted_request_digest"] = json!("sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"))),
        ("payer acceptance after expiry", Box::new(|v| v["scenarios"][0]["payer_decision"]["observed_at"] = json!("2026-11-06T13:00:01Z"))),
        ("payer fee not acknowledged", Box::new(|v| v["scenarios"][0]["payer_decision"]["acknowledged_fields"].as_array_mut().unwrap().retain(|x| x != "fee"))),
        ("untrusted local policy issuer", Box::new(|v| v["scenarios"][0]["assertions"]["local_policy_decision"]["issuer"] = json!("UNLISTED-POLICY"))),
        ("fraud signal subject mismatch", Box::new(|v| v["scenarios"][0]["assertions"]["fraud_signal"]["subject_digest"] = json!("sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"))),
        ("instruction created after decline", Box::new(|v| { v["scenarios"][0]["payer_decision"]["decision"] = json!("decline"); v["scenarios"][0]["payer_decision"]["accepted_request_digest"] = Value::Null; v["scenarios"][0]["events"] = json!([{"event_id":"MUT-1","sequence":1,"event_type":"instruction_created","actor":"SYNTH-PSP-A","authority_state":"valid","subject_digest":"sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","instruction_id":"MUT-INS","evidence_type":"payment_instruction_receipt","observed_at":"2026-11-06T12:20:01Z"}]); v["scenarios"][0]["expected_disposition"] = json!("no_instruction"); v["scenarios"][0]["expected_effect_state"] = json!("instruction_created"); })),
        ("recipient availability without instruction", Box::new(|v| { v["scenarios"][0]["events"] = json!([{"event_id":"MUT-2","sequence":1,"event_type":"recipient_available","actor":"SYNTH-PSP-B","authority_state":"valid","subject_digest":"sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","instruction_id":"MUT-INS","evidence_type":"recipient_availability_receipt","observed_at":"2026-11-06T12:20:02Z"}]); v["scenarios"][0]["expected_effect_state"] = json!("recipient_available"); })),
        ("settlement without recipient availability", Box::new(|v| { v["scenarios"][0]["events"] = json!([{"event_id":"MUT-3","sequence":1,"event_type":"instruction_created","actor":"SYNTH-PSP-A","authority_state":"valid","subject_digest":"sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","instruction_id":"MUT-INS","evidence_type":"payment_instruction_receipt","observed_at":"2026-11-06T12:20:01Z"},{"event_id":"MUT-4","sequence":2,"event_type":"settlement_observed","actor":"SYNTH-SETTLEMENT-AUTHORITY","authority_state":"valid","subject_digest":"sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","instruction_id":"MUT-INS","evidence_type":"settlement_statement_record","observed_at":"2026-11-06T12:20:02Z"}]); v["scenarios"][0]["expected_effect_state"] = json!("settlement_observed"); })),
        ("future-dated source observation", Box::new(|v| v["source_register"][0]["observed_on"] = json!("2027-01-01"))),
        ("unresolved profile source", Box::new(|v| v["profiles"][0]["source_refs"][0] = json!("MISSING"))),
        ("unknown claim promoted to verified", Box::new(|v| v["scenarios"][0]["assertions"]["collection_authority"]["state"] = json!("unknown"))),
        ("duplicate instruction", Box::new(|v| { v["scenarios"][1]["events"].push(v["scenarios"][1]["events"][0].clone()); v["scenarios"][1]["events"][1]["event_id"] = json!("DUP-INSTRUCTION"); v["scenarios"][1]["events"][1]["sequence"] = json!(2); })),
        ("finality without settlement", Box::new(|v| { v["scenarios"][1]["events"].push(json!({"event_id":"MUT-FINALITY","sequence":2,"event_type":"finality_qualified","actor":"SYNTH-PSP-A","authority_state":"valid","subject_digest":"sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","instruction_id":"INS-2","evidence_type":"profile_finality_certificate","observed_at":"2026-11-06T12:20:02Z"})); v["scenarios"][1]["expected_effect_state"] = json!("finality_qualified"); }))
    ];
    for (label, mutation) in mutations {
        mutate_and_expect_rejection(label, corpus, mutation)?;
        count += 1;
    }
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
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, "--corpus requires a path")
                })?);
                corpus_text = fs::read_to_string(path)?;
            }
            "--schema" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, "--schema requires a path")
                })?);
                schema_text = fs::read_to_string(path)?;
            }
            "--self-test" => self_test = true,
            "--help" | "-h" => {
                println!("payment-initiation-composition-evaluator [--corpus PATH] [--schema PATH] [--self-test]");
                return Ok(());
            }
            other => return Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("unknown argument: {other}")).into()),
        }
    }
    let corpus: Value = serde_json::from_str(&corpus_text)?;
    let schema: Value = serde_json::from_str(&schema_text)?;
    let mut errors = semantic_errors(&corpus);
    errors.extend(schema_errors(&schema, &corpus));
    let negative_controls = if self_test { run_self_tests(&corpus)? } else { 0 };
    let mut disposition_counts = BTreeMap::<String, u64>::new();
    let mut effect_counts = BTreeMap::<String, u64>::new();
    let mut cases = Vec::new();
    for scenario in array_at(&corpus, "scenarios").into_iter().flatten() {
        let (disposition, effect, case_errors, assurance_vector) = validate_scenario(&corpus, scenario);
        *disposition_counts.entry(disposition.clone()).or_default() += 1;
        *effect_counts.entry(effect.clone()).or_default() += 1;
        cases.push(json!({
            "scenario_id": str_at(scenario, "scenario_id"),
            "disposition": disposition,
            "effect_state": effect,
            "assurance_vector": assurance_vector,
            "errors": case_errors
        }));
    }
    let report = json!({
        "evaluator": "mycelix.payment-initiation-composition-evaluator.v1",
        "status": if errors.is_empty() { "PASS" } else { "FAIL" },
        "external_effects_enabled": false,
        "claim_ceiling": "synthetic cross-component composition only",
        "counts": {
            "scenarios": cases.len(),
            "sources": array_at(&corpus, "source_register").map_or(0, Vec::len),
            "profiles": array_at(&corpus, "profiles").map_or(0, Vec::len),
            "issuer_registrations": array_at(&corpus, "issuer_registry").map_or(0, Vec::len),
            "negative_controls": negative_controls,
            "dispositions": disposition_counts,
            "effect_states": effect_counts
        },
        "global_trust_score": Value::Null,
        "errors": errors,
        "cases": cases
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if !report["status"].as_str().is_some_and(|value| value == "PASS") {
        std::process::exit(1);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn schema_and_composition_corpus_validate() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus JSON");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema JSON");
        assert!(semantic_errors(&corpus).is_empty(), "{:?}", semantic_errors(&corpus));
        assert!(schema_errors(&schema, &corpus).is_empty(), "{:?}", schema_errors(&schema, &corpus));
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 35);
    }

    #[test]
    fn negative_mutation_campaign_rejects_cross_domain_laundering() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus JSON");
        assert_eq!(run_self_tests(&corpus).expect("mutation campaign"), 20);
    }

    #[test]
    fn no_scalar_trust_score_is_emitted() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus JSON");
        let scenario = &corpus["scenarios"][0];
        let (_, _, _, vector) = validate_scenario(&corpus, scenario);
        assert!(vector["global_trust_score"].is_null());
        assert_ne!(vector["message_authenticity"], vector["requester_identity"]);
    }
}
