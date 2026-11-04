#![forbid(unsafe_code)]

//! Effects-disabled evaluator for South African Request-to-Pay semantics.
//! A request message is not a payment. Acceptance authorises at most one instruction.

use jsonschema::draft202012;
use serde_json::{Map, Value, json};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-rtp-corpus-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-rtp-corpus-v1.schema.json");
const EXPECTED_SCHEMA: &str = "mycelix.sa-payment-rtp/v1";
const EXPECTED_CORPUS: &str = "ZA-SA-PAYMENT-RTP-SYNTHETIC-V1";

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

fn date_le(left: &str, right: &str) -> bool {
    valid_date(left) && valid_date(right) && left <= right
}

fn valid_timestamp(value: &str) -> bool {
    value.len() >= 20
        && valid_date(&value[..10])
        && value.as_bytes().get(10) == Some(&b'T')
        && (value.ends_with('Z') || value.contains('+') || value.rfind('-').is_some_and(|i| i > 10))
}

fn event_type(event: &Value) -> &str {
    str_at(event, "event_type").unwrap_or("")
}

fn canonical_json(value: &Value) -> Value {
    match value {
        Value::Object(object) => {
            let mut keys: Vec<&String> = object.keys().collect();
            keys.sort();
            let mut sorted = Map::new();
            for key in keys {
                sorted.insert(key.clone(), canonical_json(&object[key]));
            }
            Value::Object(sorted)
        }
        Value::Array(items) => Value::Array(items.iter().map(canonical_json).collect()),
        other => other.clone(),
    }
}

fn compute_request_digest(request: &Value) -> Result<String, String> {
    let mut payload = request.clone();
    let object = payload
        .as_object_mut()
        .ok_or_else(|| "request payload must be a JSON object".to_string())?;
    if object.remove("request_digest").is_none() {
        return Err("request_digest field is required before hashing".into());
    }
    let bytes = serde_json::to_vec(&canonical_json(&payload))
        .map_err(|error| format!("canonical request serialization failed: {error}"))?;
    let mut hasher = Sha256::new();
    hasher.update(b"MYCELIX-SA-PAYMENT-RTP-REQUEST-V1\0");
    hasher.update(bytes);
    Ok(format!("sha256:{:x}", hasher.finalize()))
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
    if corpus
        .get("external_effects_enabled")
        .and_then(Value::as_bool)
        != Some(false)
    {
        errors.push("external_effects_enabled: must be false".into());
    }
    let digest_contract = &corpus["request_digest_contract"];
    if str_at(digest_contract, "algorithm") != Some("SHA-256")
        || str_at(digest_contract, "domain_separator")
            != Some("MYCELIX-SA-PAYMENT-RTP-REQUEST-V1")
        || str_at(digest_contract, "separator_terminator") != Some("NUL")
        || str_at(digest_contract, "canonicalization") != Some("MYCELIX-CANONICAL-JSON-V1")
        || str_at(digest_contract, "hashed_payload")
            != Some("complete request object with request_digest omitted")
    {
        errors.push("request_digest_contract: unsupported or missing commitment contract".into());
    }

    if let Some(vectors) = array_at(digest_contract, "vectors") {
        for (index, vector) in vectors.iter().enumerate() {
            let scenario_id = str_at(vector, "scenario_id").unwrap_or("");
            let expected = str_at(vector, "expected_digest").unwrap_or("");
            let scenario = array_at(corpus, "scenarios").and_then(|items| {
                items.iter().find(|item| str_at(item, "scenario_id") == Some(scenario_id))
            });
            match scenario.and_then(|item| compute_request_digest(&item["request"]).ok()) {
                Some(actual) if actual == expected => {}
                _ => errors.push(format!(
                    "request_digest_contract.vectors[{index}]: reference digest mismatch"
                )),
            }
        }
    } else {
        errors.push("request_digest_contract.vectors: missing reference vectors".into());
    }

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
        .filter_map(|item| str_at(item, "id").map(str::to_owned))
        .collect();
    let profile_map: std::collections::BTreeMap<String, Value> = profiles
        .iter()
        .filter_map(|item| str_at(item, "profile_id").map(|id| (id.to_owned(), item.clone())))
        .collect();

    if sources.is_empty() || profiles.is_empty() || scenarios.is_empty() {
        errors.push("source_register, profiles and scenarios must be non-empty".into());
    }

    for (index, source) in sources.iter().enumerate() {
        let Some(observed) = str_at(source, "observed_on") else {
            errors.push(format!("source_register[{index}].observed_on: missing"));
            continue;
        };
        if !date_le(observed, as_of) {
            errors.push(format!(
                "source_register[{index}].observed_on: invalid or after as_of"
            ));
        }
        if let Some(published) = source.get("published_on").and_then(Value::as_str)
            && !date_le(published, as_of)
        {
            errors.push(format!(
                "source_register[{index}].published_on: invalid or after as_of"
            ));
        }
    }

    for (index, profile) in profiles.iter().enumerate() {
        let observed = str_at(profile, "observed_on").unwrap_or("");
        if !date_le(observed, as_of) {
            errors.push(format!(
                "profiles[{index}].observed_on: invalid or after as_of"
            ));
        }
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
        if let Some(start) = profile.get("effective_from").and_then(Value::as_str)
            && !valid_date(start)
        {
            errors.push(format!("profiles[{index}].effective_from: invalid date"));
        }
        if let Some(end) = profile.get("effective_to").and_then(Value::as_str)
            && !valid_date(end)
        {
            errors.push(format!("profiles[{index}].effective_to: invalid date"));
        }
    }

    for (case_index, case) in scenarios.iter().enumerate() {
        let request = &case["request"];
        let case_id = str_at(case, "scenario_id").unwrap_or("<missing>");
        let profile_id = str_at(request, "profile_id").unwrap_or("");
        let Some(profile) = profile_map.get(profile_id) else {
            errors.push(format!(
                "scenarios[{case_index}]: unresolved profile {profile_id}"
            ));
            continue;
        };

        let request_id = str_at(request, "request_id").unwrap_or("");
        let digest = str_at(request, "request_digest").unwrap_or("");
        let recomputed_digest = match compute_request_digest(request) {
            Ok(value) => value,
            Err(error) => {
                errors.push(format!("scenarios[{case_index}]: {error}"));
                String::new()
            }
        };
        if digest != recomputed_digest {
            errors.push(format!(
                "scenarios[{case_index}]: declared request digest does not commit to request fields"
            ));
        }
        let issued_at = str_at(request, "issued_at").unwrap_or("");
        let expires_at = str_at(request, "expiry_at").unwrap_or("");
        let amount = u64_at(request, "amount_minor_units").unwrap_or(0);
        if request_id.is_empty() || amount == 0 || digest.len() != 71 {
            errors.push(format!(
                "scenarios[{case_index}]: invalid request identity, amount or digest"
            ));
        }
        if !valid_timestamp(issued_at) || !valid_timestamp(expires_at) || issued_at >= expires_at {
            errors.push(format!(
                "scenarios[{case_index}]: invalid request issue/expiry interval"
            ));
        }

        if let Some(maximum) = u64_at(profile, "max_request_minor_units")
            && amount > maximum
            && !array_at(case, "events").is_some_and(|events| {
                events
                    .iter()
                    .any(|event| event_type(event) == "request_rejected_by_participant_limit")
            })
        {
            errors.push(format!(
                "scenarios[{case_index}]: amount exceeds profile limit without explicit rejection"
            ));
        }

        let events = array_at(case, "events").cloned().unwrap_or_default();
        let mut event_ids = BTreeSet::new();
        let mut previous_sequence = 0_u64;
        let mut delivered = false;
        let mut viewed = false;
        let mut terms_mutated = false;
        let mut cancelled = false;
        let mut expired = false;
        let mut declined = false;
        let mut conflict = false;
        let mut acceptance_sequence: Option<u64> = None;
        let mut instruction_id: Option<String> = None;
        let mut instruction_count = 0_u32;
        let mut provider_unknown = false;
        let mut recipient_available = false;

        for (event_index, event) in events.iter().enumerate() {
            let path = format!("scenarios[{case_index}].events[{event_index}]");
            let event_id = str_at(event, "event_id").unwrap_or("<missing>");
            if !event_ids.insert(event_id.to_owned()) {
                errors.push(format!("{path}: duplicate event_id {event_id}"));
            }
            let sequence = u64_at(event, "sequence").unwrap_or(0);
            if sequence <= previous_sequence {
                errors.push(format!("{path}: sequence must be strictly increasing"));
            }
            previous_sequence = sequence;

            let kind = event_type(event);
            let actor = str_at(event, "actor_role").unwrap_or("");
            let authority = str_at(event, "authority_state").unwrap_or("");
            let event_digest = str_at(event, "request_digest").unwrap_or("");
            let accepted_digest = str_at(event, "accepted_request_digest");
            let event_instruction = str_at(event, "instruction_id");
            let observed = str_at(event, "observed_at").unwrap_or("");
            if !valid_timestamp(observed) {
                errors.push(format!("{path}: invalid observed_at timestamp"));
            }
            if kind == "request_created" && event_digest != recomputed_digest {
                errors.push(format!(
                    "{path}: creation digest differs from immutable request digest"
                ));
            }
            if kind == "request_delivered" {
                delivered = true;
            }
            if kind == "request_viewed" {
                if !delivered {
                    errors.push(format!("{path}: request viewed before delivery"));
                }
                viewed = true;
            }
            if matches!(
                kind,
                "request_terms_mutated"
                    | "requester_identity_changed"
                    | "request_fingerprint_conflict"
            ) {
                terms_mutated = true;
            }
            if kind == "request_cancelled" {
                cancelled = true;
            }
            if kind == "request_expired"
                || (kind == "payer_acceptance_rejected" && observed >= expires_at)
            {
                expired = true;
            }
            if kind == "payer_declined" {
                declined = true;
            }
            if matches!(
                kind,
                "terms_version_conflict_detected" | "request_not_admitted_for_live_profile"
            ) {
                conflict = true;
            }

            if kind == "payer_accepted" {
                if actor != "payer" || authority != "valid" || !viewed {
                    errors.push(format!(
                        "{path}: acceptance requires viewed request, payer actor and valid authority"
                    ));
                }
                if accepted_digest != Some(recomputed_digest.as_str())
                    || event_digest != recomputed_digest
                    || terms_mutated
                {
                    errors.push(format!(
                        "{path}: acceptance is not bound to the unchanged displayed request"
                    ));
                }
                if cancelled || expired || declined || observed >= expires_at {
                    errors.push(format!(
                        "{path}: acceptance after cancel, decline or expiry"
                    ));
                }
                if acceptance_sequence.replace(sequence).is_some() {
                    errors.push(format!(
                        "{path}: multiple payer acceptance events for one request"
                    ));
                }
            }

            if kind == "payment_instruction_created" {
                instruction_count += 1;
                if instruction_count > 1 {
                    errors.push(format!(
                        "{path}: more than one payment instruction for one request"
                    ));
                }
                if let Some(accepted_at) = acceptance_sequence {
                    if sequence <= accepted_at {
                        errors.push(format!(
                            "{path}: instruction does not follow payer acceptance"
                        ));
                    }
                } else {
                    errors.push(format!(
                        "{path}: payment instruction created without payer acceptance"
                    ));
                }
                if actor != "payer_psp" || authority != "valid" {
                    errors.push(format!(
                        "{path}: instruction creation requires authorised payer PSP"
                    ));
                }
                if accepted_digest != Some(recomputed_digest.as_str())
                    || event_digest != recomputed_digest
                    || terms_mutated
                {
                    errors.push(format!(
                        "{path}: instruction does not preserve the accepted request digest"
                    ));
                }
                if cancelled || expired || declined {
                    errors.push(format!(
                        "{path}: instruction follows cancelled, expired or declined request"
                    ));
                }
                match event_instruction {
                    Some(id) if !id.is_empty() => instruction_id = Some(id.to_owned()),
                    _ => errors.push(format!("{path}: payment instruction id missing")),
                }
            }

            if matches!(
                kind,
                "duplicate_acceptance_callback_ignored" | "concurrent_instruction_suppressed"
            ) && instruction_id.as_deref() != event_instruction
            {
                errors.push(format!(
                    "{path}: suppressed duplicate does not resolve to the existing instruction"
                ));
            }

            if kind == "provider_outcome_unknown" {
                if instruction_id.as_deref() != event_instruction || authority != "unknown" {
                    errors.push(format!(
                        "{path}: unknown provider outcome is not bound to the instruction"
                    ));
                }
                provider_unknown = true;
            }
            if kind == "recipient_available" {
                if instruction_id.as_deref() != event_instruction
                    || actor != "beneficiary_psp"
                    || authority != "valid"
                {
                    errors.push(format!(
                        "{path}: recipient availability requires instruction-bound beneficiary PSP evidence"
                    ));
                }
                recipient_available = true;
            }
        }

        if instruction_count == 0 && acceptance_sequence.is_some() {
            errors.push(format!(
                "{case_id}: payer acceptance lacks a separately observed instruction result"
            ));
        }
        if instruction_count > 0 && acceptance_sequence.is_none() {
            errors.push(format!(
                "{case_id}: instruction exists without a valid acceptance"
            ));
        }
        if cancelled && instruction_count > 0 {
            errors.push(format!(
                "{case_id}: pre-acceptance cancellation created an instruction"
            ));
        }
        if expired && instruction_count > 0 {
            errors.push(format!("{case_id}: expired request created an instruction"));
        }
        if declined && instruction_count > 0 {
            errors.push(format!(
                "{case_id}: declined request created an instruction"
            ));
        }
        if conflict && instruction_count > 0 {
            errors.push(format!("{case_id}: conflicting terms version was admitted"));
        }

        let expected = if instruction_count > 0 {
            if recipient_available {
                "recipient_available"
            } else if provider_unknown {
                "outcome_unknown"
            } else {
                "instruction_created"
            }
        } else if conflict {
            "quarantined_for_requalification"
        } else if cancelled {
            "cancelled"
        } else if declined {
            "declined"
        } else if expired {
            "expired"
        } else if events
            .iter()
            .any(|event| event_type(event) == "request_rejected_by_participant_limit")
        {
            "rejected_by_limit"
        } else if events.iter().any(|event| {
            matches!(
                event_type(event),
                "request_rejected_by_channel_profile" | "profile_retired_reject"
            )
        }) {
            "rejected_by_profile"
        } else {
            "request_only"
        };

        if str_at(case, "expected_outcome") != Some(expected) {
            errors.push(format!(
                "scenarios[{case_index}]: expected_outcome does not match observed request/payment lifecycle"
            ));
        }
        let financial_effect = if recipient_available {
            "recipient_available_observed"
        } else if instruction_count == 0 {
            "none"
        } else {
            "payment_instruction_only"
        };
        if str_at(case, "expected_financial_effect") != Some(financial_effect) {
            errors.push(format!(
                "scenarios[{case_index}]: request status cannot be promoted to a financial effect"
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

    mutate_and_reject("external effects enabled", corpus, schema, |v| {
        v["external_effects_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject("instruction without acceptance", corpus, schema, |v| {
        v["scenarios"][5]["events"][3]["event_type"] = json!("request_viewed");
    })?;
    count += 1;

    mutate_and_reject("acceptance digest mismatch", corpus, schema, |v| {
        v["scenarios"][5]["events"][3]["accepted_request_digest"] =
            json!("sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
    })?;
    count += 1;

    mutate_and_reject("acceptance after expiry", corpus, schema, |v| {
        v["scenarios"][5]["events"][3]["observed_at"] = json!("2026-11-04T10:00:01Z");
    })?;
    count += 1;

    mutate_and_reject("accept after terms mutation", corpus, schema, |v| {
        v["scenarios"][6]["events"][4]["event_type"] = json!("payer_accepted");
        v["scenarios"][6]["events"][4]["authority_state"] = json!("valid");
        v["scenarios"][6]["events"][4]["accepted_request_digest"] =
            json!("sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    })?;
    count += 1;

    mutate_and_reject("duplicate instruction", corpus, schema, |v| {
        let mut instruction = v["scenarios"][5]["events"][4].clone();
        instruction["event_id"] = json!("EV-99");
        instruction["sequence"] = json!(6);
        instruction["instruction_id"] = json!("INS-DUPLICATE");
        v["scenarios"][5]["events"]
            .as_array_mut()
            .unwrap()
            .push(instruction);
    })?;
    count += 1;

    mutate_and_reject(
        "cancelled request creates instruction",
        corpus,
        schema,
        |v| {
            let mut accepted = v["scenarios"][5]["events"][3].clone();
            accepted["event_id"] = json!("EV-4");
            accepted["sequence"] = json!(4);
            let mut instruction = v["scenarios"][5]["events"][4].clone();
            instruction["event_id"] = json!("EV-5");
            instruction["sequence"] = json!(5);
            v["scenarios"][4]["events"]
                .as_array_mut()
                .unwrap()
                .push(accepted);
            v["scenarios"][4]["events"]
                .as_array_mut()
                .unwrap()
                .push(instruction);
        },
    )?;
    count += 1;

    mutate_and_reject("unknown source profile", corpus, schema, |v| {
        v["scenarios"][0]["request"]["profile_id"] = json!("ZA-MISSING-PROFILE-V1");
    })?;
    count += 1;

    mutate_and_reject("future profile observation", corpus, schema, |v| {
        v["profiles"][0]["observed_on"] = json!("2027-01-01");
    })?;
    count += 1;

    mutate_and_reject(
        "request amount above limit without rejection",
        corpus,
        schema,
        |v| {
            v["scenarios"][18]["events"][0]["event_type"] = json!("request_created");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "declined request promoted to instruction",
        corpus,
        schema,
        |v| {
            let mut accepted = v["scenarios"][5]["events"][3].clone();
            accepted["event_id"] = json!("EV-5");
            accepted["sequence"] = json!(5);
            let mut instruction = v["scenarios"][5]["events"][4].clone();
            instruction["event_id"] = json!("EV-6");
            instruction["sequence"] = json!(6);
            v["scenarios"][2]["events"]
                .as_array_mut()
                .unwrap()
                .push(accepted);
            v["scenarios"][2]["events"]
                .as_array_mut()
                .unwrap()
                .push(instruction);
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
                println!("sa-payment-rtp-evaluator [--corpus PATH] [--schema PATH] [--self-test]");
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
        "evaluator": "mycelix.sa-payment-rtp-evaluator.v1",
        "status": status,
        "external_effects_enabled": false,
        "claim_ceiling": "synthetic request/consent/profile/schema conformance only",
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
        let (errors, corpus, _) = evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("evaluation");
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(array_at(&corpus, "profiles").unwrap().len(), 5);
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 32);
    }

    #[test]
    fn request_digest_is_bound_to_amount() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let mut changed = corpus.clone();
        changed["scenarios"][5]["request"]["amount_minor_units"] = json!(6000);
        assert!(
            !semantic_errors(&changed).is_empty(),
            "an altered amount must fail the recomputed request commitment"
        );
    }

    #[test]
    fn negative_mutation_campaign_rejects_consent_and_status_attacks() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self tests"), 11);
    }
}
