#![forbid(unsafe_code)]

//! Effects-disabled evaluator for Request-to-Pay message and transfer correlation.
//! A request response, an interbank status and a beneficiary credit are different facts.

use jsonschema::draft202012;
use serde_json::{Map, Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-rtp-link-corpus-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-rtp-link-corpus-v1.schema.json");

fn str_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn u64_at(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn array_at<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn stage(event: &Value) -> &str {
    str_at(event, "stage").unwrap_or("")
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
    hasher.update(b"MYCELIX-SA-PAYMENT-RTP-LINK-REQUEST-V1\0");
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

fn unique_ids(values: &[Value], field: &str, path: &str, errors: &mut Vec<String>) {
    let mut seen = BTreeSet::new();
    for (index, value) in values.iter().enumerate() {
        match str_at(value, field) {
            Some(id) if seen.insert(id.to_owned()) => {}
            Some(id) => errors.push(format!("{path}[{index}].{field}: duplicate {id}")),
            None => errors.push(format!("{path}[{index}].{field}: missing string")),
        }
    }
}

fn has_evidence(event: &Value, wanted: &str) -> bool {
    array_at(event, "evidence_types")
        .is_some_and(|values| values.iter().any(|item| item.as_str() == Some(wanted)))
}

fn same_link(event: &Value, request: &Value, instruction_id: Option<&str>) -> bool {
    let event_end_to_end_id = str_at(event, "end_to_end_id");
    str_at(event, "original_request_id") == str_at(request, "request_id")
        && str_at(event, "request_digest") == str_at(request, "request_digest")
        && (event_end_to_end_id.is_none()
            || event_end_to_end_id == str_at(request, "end_to_end_id"))
        && (instruction_id.is_none() || str_at(event, "payment_instruction_id") == instruction_id)
}

fn semantic_errors(corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if str_at(corpus, "schema") != Some("mycelix.sa-payment-rtp-link/v1") {
        errors.push("schema: unexpected identity".into());
    }
    if str_at(corpus, "corpus_id") != Some("SA-PAYMENT-RTP-LINK-SYNTHETIC-V1") {
        errors.push("corpus_id: unexpected identity".into());
    }
    if str_at(corpus, "status") != Some("synthetic_reference_only") {
        errors.push("status: corpus must remain synthetic_reference_only".into());
    }
    if corpus
        .get("external_effects_enabled")
        .and_then(Value::as_bool)
        != Some(false)
    {
        errors.push("external_effects_enabled must be false".into());
    }

    let sources = array_at(corpus, "source_register")
        .cloned()
        .unwrap_or_default();
    let profiles = array_at(corpus, "profiles").cloned().unwrap_or_default();
    let scenarios = array_at(corpus, "scenarios").cloned().unwrap_or_default();
    unique_ids(&sources, "id", "source_register", &mut errors);
    unique_ids(&profiles, "profile_id", "profiles", &mut errors);
    unique_ids(&scenarios, "scenario_id", "scenarios", &mut errors);

    let source_ids: BTreeSet<String> = sources
        .iter()
        .filter_map(|source| str_at(source, "id").map(str::to_owned))
        .collect();
    let profile_ids: BTreeSet<String> = profiles
        .iter()
        .filter_map(|profile| str_at(profile, "profile_id").map(str::to_owned))
        .collect();

    for (index, profile) in profiles.iter().enumerate() {
        let references = array_at(profile, "source_refs")
            .cloned()
            .unwrap_or_default();
        if references.is_empty() {
            errors.push(format!("profiles[{index}].source_refs must not be empty"));
        }
        for reference in references {
            if !reference.as_str().is_some_and(|id| source_ids.contains(id)) {
                errors.push(format!(
                    "profiles[{index}].source_refs contains unknown source"
                ));
            }
        }
    }

    for (case_index, case) in scenarios.iter().enumerate() {
        let case_id = str_at(case, "scenario_id").unwrap_or("<missing>");
        let request = &case["request"];
        let request_id = str_at(request, "request_id").unwrap_or("");
        let request_digest = str_at(request, "request_digest").unwrap_or("");
        let end_to_end_id = str_at(request, "end_to_end_id").unwrap_or("");
        let amount = u64_at(request, "amount_minor_units").unwrap_or(0);
        let currency = str_at(request, "currency").unwrap_or("");
        let requester_participant = str_at(request, "requester_participant").unwrap_or("");
        let payer_participant = str_at(request, "payer_participant").unwrap_or("");
        let profile_id = str_at(case, "profile_id").unwrap_or("");

        if !profile_ids.contains(profile_id) {
            errors.push(format!(
                "scenarios[{case_index}]: unknown profile {profile_id}"
            ));
        }
        if request_id.is_empty()
            || request_digest.len() != 71
            || !request_digest.starts_with("sha256:")
            || end_to_end_id.is_empty()
            || amount == 0
            || currency.len() != 3
        {
            errors.push(format!(
                "scenarios[{case_index}]: invalid request identity or money fields"
            ));
        }
        match compute_request_digest(request) {
            Ok(expected) if expected == request_digest => {}
            Ok(_) => errors.push(format!(
                "scenarios[{case_index}]: request digest does not match canonical request fields"
            )),
            Err(error) => errors.push(format!(
                "scenarios[{case_index}]: cannot recompute request digest: {error}"
            )),
        }

        let events = array_at(case, "events").cloned().unwrap_or_default();
        let mut event_ids = BTreeSet::new();
        let mut previous_sequence = 0_u64;
        let mut request_created = false;
        let mut accepted_response = false;
        let mut instruction_id: Option<String> = None;
        let mut instruction_count = 0_u32;
        let mut valid_transfers: Vec<Value> = Vec::new();
        let mut transfer_message_count = 0_u32;
        let mut transfer_unmatched = false;
        let mut quarantined = false;
        let mut status_pending = false;
        let mut status_accepted = false;
        let mut status_rejected = false;
        let mut status_unmatched = false;
        let mut recipient_credit = false;
        let mut settlement_finality = false;
        let mut cancellation_after_instruction = false;

        let mut transfer_request_ids: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
        let mut request_transaction_ids: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
        for event in &events {
            if stage(event) == "credit_transfer_sent" {
                if let (Some(tx), Some(original)) = (
                    str_at(event, "transaction_id"),
                    str_at(event, "original_request_id"),
                ) {
                    transfer_request_ids
                        .entry(tx.to_owned())
                        .or_default()
                        .insert(original.to_owned());
                    request_transaction_ids
                        .entry(original.to_owned())
                        .or_default()
                        .insert(tx.to_owned());
                }
            }
        }
        let ambiguous_transfer_mapping = transfer_request_ids.values().any(|ids| ids.len() > 1)
            || request_transaction_ids.values().any(|ids| ids.len() > 1);

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

            let event_stage = stage(event);
            let actor = str_at(event, "actor_role").unwrap_or("");
            let participant = str_at(event, "participant_id").unwrap_or("");
            let auth = str_at(event, "authentication_state").unwrap_or("");
            let authority = str_at(event, "authority_state").unwrap_or("");

            if event_stage == "request_created" {
                request_created = true;
                if actor != "payee_rtp_service_provider"
                    || participant != requester_participant
                    || str_at(event, "message_type") != Some("pain.013.001.12")
                    || str_at(event, "original_request_id") != Some(request_id)
                    || str_at(event, "request_digest") != Some(request_digest)
                    || u64_at(event, "amount_minor_units") != Some(amount)
                    || str_at(event, "currency") != Some(currency)
                    || auth != "valid"
                    || authority != "valid"
                    || !has_evidence(event, "sealed_pain013_message")
                {
                    errors.push(format!(
                        "{path}: pain.013 creation lacks exact payee-side origin binding"
                    ));
                    quarantined = true;
                }
            }

            if event_stage == "request_delivered" {
                if !request_created
                    || actor != "payer_rtp_service_provider"
                    || participant != payer_participant
                    || str_at(event, "message_type") != Some("pain.013.001.12")
                    || !same_link(event, request, None)
                    || auth != "valid"
                    || !has_evidence(event, "inter_provider_delivery_receipt")
                {
                    errors.push(format!(
                        "{path}: request delivery lacks payer-side provider and request linkage"
                    ));
                    quarantined = true;
                }
            }

            if event_stage == "request_response" {
                if str_at(event, "message_type") != Some("pain.014.001.12")
                    || actor != "payer_rtp_service_provider"
                    || participant != payer_participant
                    || !same_link(event, request, None)
                    || auth != "valid"
                    || authority != "valid"
                    || !has_evidence(event, "pain014_response_receipt")
                {
                    errors.push(format!("{path}: pain.014 response lacks participant, authentication or request linkage"));
                    quarantined = true;
                } else if str_at(event, "status_semantics") == Some("request_accepted") {
                    accepted_response = true;
                }
            }

            if event_stage == "payment_instruction_created" {
                instruction_count += 1;
                if instruction_count > 1 {
                    errors.push(format!(
                        "{path}: one RTP request created multiple payment instructions"
                    ));
                    quarantined = true;
                }
                let candidate = str_at(event, "payment_instruction_id");
                if !accepted_response
                    || actor != "payer_rtp_service_provider"
                    || participant != payer_participant
                    || !same_link(event, request, candidate)
                    || candidate.is_none()
                    || auth != "valid"
                    || authority != "valid"
                    || u64_at(event, "amount_minor_units") != Some(amount)
                    || str_at(event, "currency") != Some(currency)
                    || !has_evidence(event, "payer_authorization_receipt")
                    || !has_evidence(event, "instruction_creation_receipt")
                {
                    errors.push(format!("{path}: payment instruction lacks exact accepted request and payer authority"));
                    quarantined = true;
                } else {
                    instruction_id = candidate.map(str::to_owned);
                }
            }

            if event_stage == "credit_transfer_sent" {
                transfer_message_count += 1;
                let transaction_id = str_at(event, "transaction_id");
                let transfer_link_ok = instruction_id.is_some()
                    && accepted_response
                    && same_link(event, request, instruction_id.as_deref())
                    && str_at(event, "end_to_end_id") == Some(end_to_end_id)
                    && u64_at(event, "amount_minor_units") == Some(amount)
                    && str_at(event, "currency") == Some(currency);
                if actor != "debtor_agent"
                    || participant != payer_participant
                    || str_at(event, "message_type") != Some("pacs.008")
                    || auth != "valid"
                    || authority != "valid"
                    || !has_evidence(event, "authenticated_pacs008")
                    || !has_evidence(event, "interbank_transfer_message")
                {
                    quarantined = true;
                } else if !transfer_link_ok {
                    transfer_unmatched = true;
                } else {
                    valid_transfers.push(event.clone());
                }
                if transaction_id.is_none() {
                    errors.push(format!("{path}: pacs.008 lacks transaction_id"));
                    transfer_unmatched = true;
                }
            }

            if event_stage == "interbank_status_report" {
                let transaction_id = str_at(event, "transaction_id");
                let matching_transfer = valid_transfers.iter().any(|transfer| {
                    str_at(transfer, "transaction_id") == transaction_id
                        && same_link(event, request, instruction_id.as_deref())
                        && str_at(event, "end_to_end_id") == Some(end_to_end_id)
                });
                if actor != "creditor_agent"
                    || participant != requester_participant
                    || str_at(event, "message_type") != Some("pacs.002")
                    || auth != "valid"
                    || authority != "valid"
                    || !has_evidence(event, "authenticated_pacs002")
                    || !has_evidence(event, "original_instruction_reference")
                {
                    quarantined = true;
                } else if !matching_transfer {
                    status_unmatched = true;
                } else {
                    match str_at(event, "status_semantics").unwrap_or("") {
                        "pending" => status_pending = true,
                        "instruction_accepted" => status_accepted = true,
                        "transfer_rejected" => status_rejected = true,
                        _ => errors.push(format!(
                            "{path}: unknown status semantics must remain indeterminate"
                        )),
                    }
                }
            }

            if event_stage == "recipient_credit_confirmed" {
                let transaction_id = str_at(event, "transaction_id");
                let matching_transfer = valid_transfers.iter().any(|transfer| {
                    str_at(transfer, "transaction_id") == transaction_id
                        && same_link(event, request, instruction_id.as_deref())
                        && str_at(event, "end_to_end_id") == Some(end_to_end_id)
                        && u64_at(event, "amount_minor_units") == Some(amount)
                        && str_at(event, "currency") == Some(currency)
                });
                if actor != "creditor_agent"
                    || participant != requester_participant
                    || auth != "valid"
                    || authority != "valid"
                    || !matching_transfer
                    || !has_evidence(event, "beneficiary_account_credit_receipt")
                    || !has_evidence(event, "creditor_agent_confirmation")
                {
                    errors.push(format!(
                        "{path}: recipient credit lacks transfer-bound creditor-agent evidence"
                    ));
                    quarantined = true;
                } else {
                    recipient_credit = true;
                }
            }

            if event_stage == "settlement_confirmed" {
                let transaction_id = str_at(event, "transaction_id");
                let matching_transfer = valid_transfers.iter().any(|transfer| {
                    str_at(transfer, "transaction_id") == transaction_id
                        && same_link(event, request, instruction_id.as_deref())
                        && str_at(event, "end_to_end_id") == Some(end_to_end_id)
                        && u64_at(event, "amount_minor_units") == Some(amount)
                        && str_at(event, "currency") == Some(currency)
                });
                if actor != "settlement_operator"
                    || auth != "valid"
                    || authority != "valid"
                    || !matching_transfer
                    || !recipient_credit
                    || !has_evidence(event, "settlement_finality_receipt")
                    || !has_evidence(event, "settlement_operator_confirmation")
                {
                    errors.push(format!(
                        "{path}: settlement finality lacks transfer and recipient-credit evidence"
                    ));
                    quarantined = true;
                } else {
                    settlement_finality = true;
                }
            }

            if event_stage == "request_cancelled" && instruction_count > 0 {
                cancellation_after_instruction = true;
            }
        }

        let distinct_transfer_ids: BTreeSet<String> = valid_transfers
            .iter()
            .filter_map(|transfer| str_at(transfer, "transaction_id").map(str::to_owned))
            .collect();

        let computed_state = if quarantined {
            "quarantined"
        } else if ambiguous_transfer_mapping {
            "ambiguous_link"
        } else if transfer_unmatched {
            "transfer_unmatched"
        } else if status_unmatched {
            "status_unmatched"
        } else if settlement_finality {
            "settled_final"
        } else if recipient_credit {
            "recipient_available"
        } else if status_rejected {
            "transfer_rejected"
        } else if status_pending {
            "transfer_pending"
        } else if status_accepted {
            "transfer_status_only"
        } else if transfer_message_count > 1 && distinct_transfer_ids.len() == 1 {
            "one_transfer_idempotent"
        } else if cancellation_after_instruction {
            "cancel_after_instruction"
        } else if instruction_count > 0 && valid_transfers.is_empty() {
            "instruction_created"
        } else if !valid_transfers.is_empty() {
            "transfer_unmatched"
        } else if accepted_response && instruction_count == 0 {
            "accepted_no_instruction"
        } else {
            "request_only"
        };

        if str_at(case, "expected_state") != Some(computed_state) {
            errors.push(format!(
                "scenarios[{case_index}]: expected_state disagrees with evidence graph ({computed_state})"
            ));
        }

        if case_id.is_empty() {
            errors.push(format!(
                "scenarios[{case_index}]: missing scenario identity"
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
    let mut errors = semantic_errors(corpus);
    errors.extend(schema_errors(schema, corpus));
    if !errors.is_empty() {
        return Err(format!(
            "baseline corpus must pass before mutations: {errors:?}"
        ));
    }
    let mut count = 0;

    mutate_and_reject("external effects enabled", corpus, schema, |value| {
        value["external_effects_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject(
        "request digest no longer matches canonical fields",
        corpus,
        schema,
        |value| {
            value["scenarios"][12]["request"]["request_digest"] =
                json!("sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "transfer without payer instruction",
        corpus,
        schema,
        |value| {
            value["scenarios"][12]["events"][3]["stage"] = json!("request_viewed");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "pacs002 accepted status promoted to settlement",
        corpus,
        schema,
        |value| {
            value["scenarios"][4]["expected_state"] = json!("settled_final");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "pacs008 amount mismatch accepted",
        corpus,
        schema,
        |value| {
            value["scenarios"][12]["events"][4]["amount_minor_units"] = json!(1);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "wrong participant role for pacs008",
        corpus,
        schema,
        |value| {
            value["scenarios"][12]["events"][4]["actor_role"] = json!("payee_rtp_service_provider");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "revoked key accepted for pacs008",
        corpus,
        schema,
        |value| {
            value["scenarios"][12]["events"][4]["authentication_state"] = json!("revoked");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "pacs002 with no matching pacs008 accepted",
        corpus,
        schema,
        |value| {
            value["scenarios"][24]["expected_state"] = json!("transfer_status_only");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "request response replay creates a second instruction",
        corpus,
        schema,
        |value| {
            let mut instruction = value["scenarios"][12]["events"][3].clone();
            instruction["event_id"] = json!("EV-9");
            instruction["sequence"] = json!(9);
            instruction["payment_instruction_id"] = json!("INS-SECOND");
            value["scenarios"][12]["events"]
                .as_array_mut()
                .unwrap()
                .push(instruction);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "recipient credit inferred from pacs002 alone",
        corpus,
        schema,
        |value| {
            value["scenarios"][4]["expected_state"] = json!("recipient_available");
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
                println!(
                    "sa-payment-rtp-link-evaluator [--corpus PATH] [--schema PATH] [--self-test]"
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
    let negative_controls = if self_test {
        run_self_tests(&corpus, &schema)?
    } else {
        0
    };
    let status = if errors.is_empty() { "PASS" } else { "FAIL" };
    let report = json!({
        "evaluator": "mycelix.sa-payment-rtp-link-evaluator.v1",
        "status": status,
        "external_effects_enabled": false,
        "claim_ceiling": "synthetic correlation/evidence conformance only",
        "source_count": array_at(&corpus, "source_register").map_or(0, Vec::len),
        "profile_count": array_at(&corpus, "profiles").map_or(0, Vec::len),
        "scenario_count": array_at(&corpus, "scenarios").map_or(0, Vec::len),
        "negative_controls": negative_controls,
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
        assert_eq!(array_at(&corpus, "source_register").unwrap().len(), 7);
        assert_eq!(array_at(&corpus, "profiles").unwrap().len(), 3);
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 26);
    }

    #[test]
    fn negative_mutations_reject_false_correlation_and_status_promotion() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self tests"), 10);
    }
}
