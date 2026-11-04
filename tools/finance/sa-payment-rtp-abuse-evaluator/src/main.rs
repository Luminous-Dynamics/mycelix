#![forbid(unsafe_code)]

//! Effects-disabled evaluator for RTP requester authenticity and abuse boundaries.
//! Sender authentication, identity, collection authority, commercial evidence and payer choice
//! remain separate claims. No scenario in this corpus can create a payment instruction.

use jsonschema::draft202012;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-rtp-abuse-corpus-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-rtp-abuse-corpus-v1.schema.json");
const EXPECTED_SCHEMA: &str = "mycelix.sa-payment-rtp-abuse/v1";
const EXPECTED_CORPUS: &str = "ZA-SA-PAYMENT-RTP-ABUSE-SYNTHETIC-V1";

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

fn presentation_complete(presentation: &Value) -> bool {
    if str_at(presentation, "accessibility_state") != Some("qualified") {
        return false;
    }
    let required = array_at(presentation, "required_material_fields");
    let displayed = array_at(presentation, "displayed_material_fields");
    let (Some(required), Some(displayed)) = (required, displayed) else {
        return false;
    };
    let displayed_set: BTreeSet<&str> = displayed.iter().filter_map(Value::as_str).collect();
    if !required
        .iter()
        .filter_map(Value::as_str)
        .all(|field| displayed_set.contains(field))
    {
        return false;
    }
    if presentation.get("ai_summary_used").and_then(Value::as_bool) == Some(true) {
        let Some(summary) = array_at(presentation, "ai_summary_fields") else {
            return false;
        };
        let summary_set: BTreeSet<&str> = summary.iter().filter_map(Value::as_str).collect();
        if !required
            .iter()
            .filter_map(Value::as_str)
            .all(|field| summary_set.contains(field))
        {
            return false;
        }
    }
    true
}

fn claim_rank(value: &str) -> Option<u8> {
    match value {
        "unproven" => Some(0),
        "sender_session_only" => Some(1),
        "account_name_match_only" => Some(2),
        "requester_identity_only" => Some(3),
        "requester_authority_supported" => Some(4),
        "commercial_evidence_present_nonlegal" => Some(5),
        _ => None,
    }
}

fn maximum_claim_rank(case: &Value) -> u8 {
    let identity = str_at(case, "requester_identity_evidence").unwrap_or("unknown");
    let authority = str_at(case, "collection_authority").unwrap_or("unknown");
    let commercial = str_at(case, "commercial_claim_evidence").unwrap_or("missing");
    let binding = str_at(case, "payee_binding").unwrap_or("unknown");
    let compromise = str_at(case, "credential_compromise_state").unwrap_or("unknown");
    let secondary = str_at(case, "secondary_data_use").unwrap_or("unknown");

    if matches!(binding, "mismatch")
        || matches!(compromise, "suspected" | "confirmed")
        || secondary != "none"
    {
        return 0;
    }

    let mut rank = match identity {
        "display_name_only"
            if str_at(case, "requester_authentication") == Some("authenticated_session") =>
        {
            1
        }
        "account_name_match" => 2,
        "verified_merchant_identity" => 3,
        _ => 0,
    };

    if matches!(binding, "ambiguous" | "confusable") {
        rank = rank.min(2);
    }
    if authority == "verified_for_beneficiary" && rank >= 3 {
        rank = 4;
    }
    if commercial == "invoice_evidence_bound"
        && authority == "verified_for_beneficiary"
        && rank >= 4
    {
        rank = 5;
    }
    rank
}

fn expected_disposition(case: &Value, profile: &Value) -> &'static str {
    let binding = str_at(case, "payee_binding").unwrap_or("unknown");
    let compromise = str_at(case, "credential_compromise_state").unwrap_or("unknown");
    let auth = str_at(case, "requester_authentication").unwrap_or("unknown");
    let request_count = u64_at(case, "request_count_24h").unwrap_or(0);
    let published_limit = profile.get("request_limit_24h").and_then(Value::as_u64);
    let limit_state = str_at(case, "limit_source_state").unwrap_or("unknown");
    let payer_action = str_at(case, "payer_action").unwrap_or("pending");
    let prior_declined = case.get("prior_declined").and_then(Value::as_bool) == Some(true);
    let authority = str_at(case, "collection_authority").unwrap_or("unknown");
    let commercial = str_at(case, "commercial_claim_evidence").unwrap_or("missing");
    let secondary = str_at(case, "secondary_data_use").unwrap_or("unknown");
    let appeal = str_at(case, "appeal_state").unwrap_or("none");

    if binding == "mismatch" {
        "reject_payee_mismatch"
    } else if matches!(compromise, "suspected" | "confirmed") || auth == "stale_session" {
        "credential_compromise_review"
    } else if limit_state == "source_backed"
        && published_limit.is_some_and(|limit| request_count > limit)
    {
        "blocked_by_published_limit"
    } else if secondary != "none" {
        "reject_secondary_data_use"
    } else if !presentation_complete(&case["presentation"]) {
        "presentation_incomplete"
    } else if commercial == "unsupported_legal_assertion" {
        "flag_unsupported_claim"
    } else if payer_action == "reported" {
        "report_no_payment_effect"
    } else if payer_action == "declined" || payer_action == "blocked" {
        "decline_no_payment_effect"
    } else if payer_action == "queried" {
        "query_no_payment_effect"
    } else if prior_declined {
        "repeat_after_decline_review"
    } else if limit_state == "unknown"
        && profile.get("request_limit_24h").is_none_or(Value::is_null)
    {
        "limit_unknown_do_not_invent_rejection"
    } else if appeal == "pending" {
        "appeal_review_not_automatic_exclusion"
    } else if matches!(binding, "ambiguous" | "confusable") {
        "manual_identity_review"
    } else if authority != "verified_for_beneficiary" {
        "identity_or_authority_unqualified"
    } else if matches!(
        commercial,
        "missing" | "stale" | "conflicting" | "merchant_record_only"
    ) {
        "commercial_claim_unsubstantiated"
    } else {
        "identity_or_authority_unqualified"
    }
}

fn write_canonical_json(value: &Value, output: &mut String) -> Result<(), serde_json::Error> {
    match value {
        Value::Null => output.push_str("null"),
        Value::Bool(value) => output.push_str(if *value { "true" } else { "false" }),
        Value::Number(value) => output.push_str(&value.to_string()),
        Value::String(value) => output.push_str(&serde_json::to_string(value)?),
        Value::Array(values) => {
            output.push('[');
            for (index, child) in values.iter().enumerate() {
                if index > 0 {
                    output.push(',');
                }
                write_canonical_json(child, output)?;
            }
            output.push(']');
        }
        Value::Object(fields) => {
            let mut keys: Vec<&String> = fields.keys().collect();
            keys.sort_unstable();
            output.push('{');
            for (index, key) in keys.iter().enumerate() {
                if index > 0 {
                    output.push(',');
                }
                output.push_str(&serde_json::to_string(key)?);
                output.push(':');
                write_canonical_json(&fields[*key], output)?;
            }
            output.push('}');
        }
    }
    Ok(())
}

fn valid_observation_commitment(observation: &Value) -> bool {
    let Some(expected) = str_at(observation, "evidence_commitment") else {
        return false;
    };
    let mut projection = observation.clone();
    let Some(fields) = projection.as_object_mut() else {
        return false;
    };
    fields.remove("evidence_commitment");
    let mut canonical = String::new();
    if write_canonical_json(&projection, &mut canonical).is_err() {
        return false;
    }
    let digest = Sha256::digest(canonical.as_bytes());
    expected == format!("sha256:{digest:x}")
}

fn has_bound_observation(
    case: &Value,
    subject_ref: &str,
    signal: &str,
    allowed_issuers: &[&str],
    allowed_states: &[&str],
    as_of: &str,
) -> bool {
    array_at(case, "risk_observations").is_some_and(|observations| {
        observations.iter().any(|observation| {
            str_at(observation, "signal_code") == Some(signal)
                && str_at(observation, "subject_binding_state") == Some("bound")
                && str_at(observation, "subject_ref") == Some(subject_ref)
                && str_at(observation, "freshness_state") == Some("current")
                && str_at(observation, "appeal_state") != Some("overturned")
                && str_at(observation, "issuer_class")
                    .is_some_and(|issuer| allowed_issuers.contains(&issuer))
                && str_at(observation, "evidence_state")
                    .is_some_and(|state| allowed_states.contains(&state))
                && observation
                    .get("expires_on")
                    .and_then(Value::as_str)
                    .is_none_or(|expiry| valid_date(expiry) && expiry > as_of)
        })
    })
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
    let profile_map: BTreeMap<String, Value> = profiles
        .iter()
        .filter_map(|item| str_at(item, "profile_id").map(|id| (id.to_owned(), item.clone())))
        .collect();

    for (index, source) in sources.iter().enumerate() {
        let observed = str_at(source, "observed_on").unwrap_or("");
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
        let source_refs = array_at(profile, "source_refs")
            .cloned()
            .unwrap_or_default();
        for reference in source_refs {
            if !reference.as_str().is_some_and(|id| source_ids.contains(id)) {
                errors.push(format!("profiles[{index}].source_refs: unresolved source"));
            }
        }
        let limit = profile.get("request_limit_24h");
        let scope = str_at(profile, "request_limit_scope").unwrap_or("unknown");
        if limit.is_none_or(Value::is_null) && scope != "unknown" && scope != "not_applicable" {
            errors.push(format!(
                "profiles[{index}]: missing request limit must retain unknown scope"
            ));
        }
        if u64_at(profile, "request_limit_24h").is_some() && scope == "unknown" {
            errors.push(format!(
                "profiles[{index}]: known limit cannot have unknown scope"
            ));
        }
    }

    let mut global_observation_ids = BTreeSet::new();

    for (index, case) in scenarios.iter().enumerate() {
        let id = str_at(case, "scenario_id").unwrap_or("<missing>");
        let profile_id = str_at(case, "profile_id").unwrap_or("");
        let Some(profile) = profile_map.get(profile_id) else {
            errors.push(format!(
                "scenarios[{index}]: unresolved profile {profile_id}"
            ));
            continue;
        };

        let source_refs = array_at(case, "source_refs").cloned().unwrap_or_default();
        for reference in source_refs {
            if !reference
                .as_str()
                .is_some_and(|source_id| source_ids.contains(source_id))
            {
                errors.push(format!("scenarios[{index}].source_refs: unresolved source"));
            }
        }

        let vop_scheme = str_at(case, "payee_verification_scheme").unwrap_or("not_performed");
        let vop_result = str_at(case, "payee_verification_result").unwrap_or("not_performed");
        let vop_guidance = str_at(case, "payee_verification_guidance");
        let payer_warning = str_at(case, "payer_warning_state").unwrap_or("not_applicable");
        let payer_choice = str_at(case, "payer_choice_state").unwrap_or("not_applicable");

        if vop_scheme == "not_performed" {
            if vop_result != "not_performed"
                || vop_guidance.is_some()
                || payer_warning != "not_applicable"
                || payer_choice != "not_applicable"
            {
                errors.push(format!(
                    "{id}: unperformed payee verification carries a result, guidance or warning"
                ));
            }
        } else if vop_scheme == "EPC_VOP_V1_1_COMPARATOR" {
            if str_at(profile, "jurisdiction") != Some("EU-SEPA") {
                errors.push(format!(
                    "{id}: EPC VOP comparator must use EU-SEPA profile, not South African rules"
                ));
            }
            if !matches!(
                vop_result,
                "match" | "close_match" | "no_match" | "verification_not_possible"
            ) {
                errors.push(format!(
                    "{id}: EPC VOP comparator has invalid or missing response outcome"
                ));
            }
            if vop_guidance != Some("EPC_VOP_MATCHING_V2.0") {
                errors.push(format!(
                    "{id}: VOP matching guidance version must be explicitly bound"
                ));
            }
            let base_vop_sources = [
                "EPC_VOP_RULEBOOK_2026",
                "EPC_VOP_MATCHING_2026",
                "EPC_VOP_SCHEME_OVERVIEW_2026",
            ];
            if base_vop_sources.iter().any(|source_id| {
                !source_refs
                    .iter()
                    .any(|item| item.as_str() == Some(source_id))
            }) {
                errors.push(format!(
                    "{id}: VOP comparator source register is incomplete"
                ));
            }
            match vop_result {
                "match"
                    if str_at(case, "requester_identity_evidence")
                        == Some("account_name_match")
                        && str_at(case, "collection_authority")
                            != Some("verified_for_beneficiary")
                        && str_at(case, "expected_claim_ceiling")
                            .and_then(claim_rank)
                            .is_some_and(|rank| rank > 2) =>
                {
                    errors.push(format!(
                        "{id}: VOP account-name match was promoted above account-name evidence"
                    ));
                }
                "close_match" => {
                    if !matches!(
                        str_at(case, "payee_binding"),
                        Some("ambiguous" | "confusable")
                    ) || str_at(case, "expected_disposition") != Some("manual_identity_review")
                    {
                        errors.push(format!(
                            "{id}: VOP close match must remain a manual identity-review state"
                        ));
                    }
                }
                "no_match" | "verification_not_possible" => {
                    let no_match_source_present = source_refs
                        .iter()
                        .any(|item| item.as_str() == Some("EPC_VOP_NO_MATCH_PAYER_CHOICE_2026"));
                    if payer_warning != "warning_displayed"
                        || payer_choice != "preserved"
                        || str_at(case, "payee_binding") == Some("mismatch")
                        || str_at(case, "expected_disposition") != Some("manual_identity_review")
                        || !no_match_source_present
                    {
                        errors.push(format!(
                            "{id}: VOP no-match/unavailable must be a source-backed warning with payer choice preserved"
                        ));
                    }
                }
                _ => {}
            }
        } else {
            errors.push(format!("{id}: unknown payee verification scheme"));
        }

        let requester_subject_ref = str_at(case, "requester_subject_ref").unwrap_or("");
        if requester_subject_ref.is_empty() {
            errors.push(format!("{id}: requester_subject_ref is missing"));
        }
        let observations = array_at(case, "risk_observations")
            .cloned()
            .unwrap_or_default();
        let mut local_observation_ids = BTreeSet::new();
        for (observation_index, observation) in observations.iter().enumerate() {
            let observation_id = str_at(observation, "observation_id").unwrap_or("<missing>");
            if !local_observation_ids.insert(observation_id.to_owned())
                || !global_observation_ids.insert(observation_id.to_owned())
            {
                errors.push(format!(
                    "{id}: duplicate risk observation id {observation_id}"
                ));
            }
            let observed_on = str_at(observation, "observed_on").unwrap_or("");
            if !valid_observation_commitment(observation) {
                errors.push(format!(
                    "{id}: risk observation {observation_id} has an invalid SHA-256 commitment"
                ));
            }
            if !date_le(observed_on, as_of) {
                errors.push(format!(
                    "{id}: risk observation {observation_id} has invalid/future observed_on"
                ));
            }
            let expires_on = observation.get("expires_on").and_then(Value::as_str);
            if let Some(expiry) = expires_on
                && (!date_le(observed_on, expiry)
                    || (date_le(expiry, as_of)
                        && str_at(observation, "freshness_state") == Some("current")))
            {
                errors.push(format!(
                    "{id}: risk observation {observation_id} has invalid or mislabelled expiry"
                ));
            }
            let evidence_state = str_at(observation, "evidence_state").unwrap_or("");
            let appeal_state = str_at(observation, "appeal_state").unwrap_or("");
            let issuer = str_at(observation, "issuer_class").unwrap_or("");
            if evidence_state == "refuted" && appeal_state != "overturned" {
                errors.push(format!(
                    "{id}: refuted observation {observation_id} lacks overturned appeal disposition"
                ));
            }
            if appeal_state == "overturned" && evidence_state != "refuted" {
                errors.push(format!(
                    "{id}: overturned observation {observation_id} is not marked refuted"
                ));
            }
            if matches!(
                issuer,
                "payer_report" | "third_party_complaint" | "automated_heuristic"
            ) && evidence_state == "independently_confirmed"
            {
                errors.push(format!(
                    "{id}: observation {observation_id} promotes a non-adjudicative source to independent confirmation"
                ));
            }
            let binding = str_at(observation, "subject_binding_state").unwrap_or("");
            let observed_subject = str_at(observation, "subject_ref").unwrap_or("");
            if binding == "bound" && observed_subject != requester_subject_ref {
                errors.push(format!(
                    "{id}: bound observation {observation_id} belongs to another subject"
                ));
            }
            if binding == "unbound" && observed_subject == requester_subject_ref {
                errors.push(format!(
                    "{id}: unbound observation {observation_id} has a matching subject commitment"
                ));
            }
            if binding != "bound"
                && str_at(case, "expected_claim_ceiling")
                    .and_then(claim_rank)
                    .is_some_and(|rank| rank > 0)
            {
                errors.push(format!(
                    "{id}: unbound risk observation cannot support a positive claim ceiling"
                ));
            }
            if str_at(observation, "purpose_scope") != Some("payment_request_abuse_only") {
                errors.push(format!(
                    "{id}: risk observation {observation_id} escaped its declared purpose scope"
                ));
            }
            if observation_index > 0 && observation_id == "<missing>" {
                errors.push(format!("{id}: risk observation identity missing"));
            }
        }

        let credential_state = str_at(case, "credential_compromise_state").unwrap_or("unknown");
        if credential_state == "suspected"
            && !has_bound_observation(
                case,
                requester_subject_ref,
                "credential_compromise",
                &["provider_security_notice", "independent_adjudication"],
                &["corroborated", "independently_confirmed"],
                as_of,
            )
        {
            errors.push(format!(
                "{id}: suspected credential compromise lacks current bound evidence"
            ));
        }
        if credential_state == "confirmed"
            && !has_bound_observation(
                case,
                requester_subject_ref,
                "credential_compromise",
                &["provider_security_notice", "independent_adjudication"],
                &["independently_confirmed"],
                as_of,
            )
        {
            errors.push(format!(
                "{id}: confirmed credential compromise lacks independent current evidence"
            ));
        }
        if case.get("prior_declined").and_then(Value::as_bool) == Some(true)
            && !has_bound_observation(
                case,
                requester_subject_ref,
                "repeat_after_decline",
                &["system_audit", "provider_security_notice", "payer_report"],
                &["corroborated", "independently_confirmed"],
                as_of,
            )
        {
            errors.push(format!(
                "{id}: repeat-after-decline signal lacks bound current evidence"
            ));
        }
        if str_at(case, "secondary_data_use").is_some_and(|value| value != "none")
            && !has_bound_observation(
                case,
                requester_subject_ref,
                "secondary_data_use",
                &["system_audit"],
                &["independently_confirmed"],
                as_of,
            )
        {
            errors.push(format!(
                "{id}: secondary data-use finding lacks independent audit evidence"
            ));
        }
        if str_at(case, "payer_action") == Some("reported")
            && !has_bound_observation(
                case,
                requester_subject_ref,
                "payer_report",
                &["payer_report"],
                &["unverified", "corroborated"],
                as_of,
            )
        {
            errors.push(format!(
                "{id}: payer report action lacks a subject-bound report receipt"
            ));
        }
        if str_at(case, "payee_binding") == Some("confusable")
            && !has_bound_observation(
                case,
                requester_subject_ref,
                "identity_confusable",
                &["automated_heuristic"],
                &["unverified", "corroborated"],
                as_of,
            )
        {
            errors.push(format!(
                "{id}: confusable identity signal lacks heuristic observation evidence"
            ));
        }
        if str_at(case, "appeal_state") == Some("reversed")
            && !observations.iter().any(|observation| {
                str_at(observation, "appeal_state") == Some("overturned")
                    && str_at(observation, "evidence_state") == Some("refuted")
            })
        {
            errors.push(format!(
                "{id}: reversed appeal lacks a superseding refutation observation"
            ));
        }

        if case.get("instruction_created").and_then(Value::as_bool) != Some(false) {
            errors.push(format!(
                "{id}: abuse corpus must not create payment instructions"
            ));
        }
        if matches!(
            str_at(case, "payer_action"),
            Some("pending" | "declined" | "queried" | "reported" | "blocked")
        ) && case.get("instruction_created").and_then(Value::as_bool) == Some(true)
        {
            errors.push(format!(
                "{id}: non-acceptance payer action created an instruction"
            ));
        }

        let limit_state = str_at(case, "limit_source_state").unwrap_or("unknown");
        let published_case_limit = case
            .get("published_request_limit_24h")
            .and_then(Value::as_u64);
        let profile_limit = profile.get("request_limit_24h").and_then(Value::as_u64);
        if limit_state == "source_backed" && published_case_limit != profile_limit {
            errors.push(format!(
                "{id}: scenario limit differs from participant profile"
            ));
        }
        if limit_state == "unknown" && (published_case_limit.is_some() || profile_limit.is_some()) {
            errors.push(format!(
                "{id}: unknown limit state conflicts with known limit value"
            ));
        }
        if limit_state == "source_backed" && published_case_limit.is_none() {
            errors.push(format!("{id}: source-backed limit is missing"));
        }

        let maximum = maximum_claim_rank(case);
        let stated = str_at(case, "expected_claim_ceiling")
            .and_then(claim_rank)
            .unwrap_or(u8::MAX);
        if stated > maximum {
            errors.push(format!(
                "{id}: claim ceiling overstates available identity/authority/commercial evidence"
            ));
        }

        let computed = expected_disposition(case, profile);
        if str_at(case, "expected_disposition") != Some(computed) {
            errors.push(format!(
                "{id}: expected disposition does not match source-bound evidence and payer state (computed {computed})"
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
    let mut baseline = semantic_errors(corpus);
    baseline.extend(schema_errors(schema, corpus));
    if !baseline.is_empty() {
        return Err(format!("baseline corpus failed: {baseline:?}"));
    }
    let mut count = 0;

    mutate_and_reject("external effects enabled", corpus, schema, |v| {
        v["external_effects_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject(
        "session authentication promoted to legal identity",
        corpus,
        schema,
        |v| {
            v["scenarios"][0]["expected_claim_ceiling"] = json!("requester_identity_only");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "account-name match promoted to collection authority",
        corpus,
        schema,
        |v| {
            v["scenarios"][1]["expected_claim_ceiling"] = json!("requester_authority_supported");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "missing collection authority promoted to authority supported",
        corpus,
        schema,
        |v| {
            v["scenarios"][18]["expected_claim_ceiling"] = json!("requester_authority_supported");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "missing invoice evidence promoted to commercial evidence",
        corpus,
        schema,
        |v| {
            v["scenarios"][3]["expected_claim_ceiling"] =
                json!("commercial_evidence_present_nonlegal");
        },
    )?;
    count += 1;

    mutate_and_reject("payee mismatch not rejected", corpus, schema, |v| {
        v["scenarios"][2]["expected_disposition"] = json!("identity_or_authority_unqualified");
    })?;
    count += 1;

    mutate_and_reject("decline creates instruction", corpus, schema, |v| {
        v["scenarios"][12]["instruction_created"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject("report creates instruction", corpus, schema, |v| {
        v["scenarios"][11]["instruction_created"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject(
        "unknown participant limit becomes hard rejection",
        corpus,
        schema,
        |v| {
            v["scenarios"][23]["expected_disposition"] = json!("blocked_by_published_limit");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "published request limit is not enforced",
        corpus,
        schema,
        |v| {
            v["scenarios"][6]["expected_disposition"] = json!("repeat_after_decline_review");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "AI summary omits fee but is treated complete",
        corpus,
        schema,
        |v| {
            v["scenarios"][17]["expected_disposition"] = json!("identity_or_authority_unqualified");
        },
    )?;
    count += 1;

    mutate_and_reject("secondary data use is accepted", corpus, schema, |v| {
        v["scenarios"][13]["expected_disposition"] = json!("identity_or_authority_unqualified");
    })?;
    count += 1;

    mutate_and_reject("source reference is unresolvable", corpus, schema, |v| {
        v["scenarios"][0]["source_refs"] = json!(["MISSING_SOURCE"]);
    })?;
    count += 1;

    mutate_and_reject("duplicate scenario identity", corpus, schema, |v| {
        v["scenarios"][1]["scenario_id"] = v["scenarios"][0]["scenario_id"].clone();
    })?;
    count += 1;

    mutate_and_reject(
        "cross-subject observation relabelled as bound",
        corpus,
        schema,
        |v| {
            v["scenarios"][27]["risk_observations"][0]["subject_binding_state"] = json!("bound");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "heuristic promoted to independent confirmation",
        corpus,
        schema,
        |v| {
            v["scenarios"][8]["risk_observations"][0]["evidence_state"] =
                json!("independently_confirmed");
        },
    )?;
    count += 1;

    mutate_and_reject("overturned observation revived", corpus, schema, |v| {
        v["scenarios"][25]["risk_observations"][0]["evidence_state"] = json!("corroborated");
    })?;
    count += 1;

    mutate_and_reject(
        "expired observation still marked current",
        corpus,
        schema,
        |v| {
            v["scenarios"][24]["risk_observations"][0]["expires_on"] = json!("2026-11-04");
            v["scenarios"][24]["risk_observations"][0]["freshness_state"] = json!("current");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "suspected credential compromise without bound evidence",
        corpus,
        schema,
        |v| {
            v["scenarios"][7]["risk_observations"] = json!([]);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "repeat-after-decline without subject-bound observation",
        corpus,
        schema,
        |v| {
            v["scenarios"][5]["risk_observations"] = json!([]);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "reported request without payer-report receipt",
        corpus,
        schema,
        |v| {
            v["scenarios"][11]["risk_observations"] = json!([]);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "secondary data use without independent audit",
        corpus,
        schema,
        |v| {
            v["scenarios"][13]["risk_observations"] = json!([]);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "risk observation commitment tampered",
        corpus,
        schema,
        |v| {
            v["scenarios"][4]["risk_observations"][0]["evidence_state"] = json!("corroborated");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "VOP account-name match promoted to legal identity",
        corpus,
        schema,
        |v| {
            v["scenarios"][28]["expected_claim_ceiling"] = json!("requester_identity_only");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "VOP close match auto-accepted without identity review",
        corpus,
        schema,
        |v| {
            v["scenarios"][29]["expected_disposition"] = json!("identity_or_authority_unqualified");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "VOP no-match converted into a payer block",
        corpus,
        schema,
        |v| {
            v["scenarios"][30]["payer_choice_state"] = json!("blocked");
        },
    )?;
    count += 1;

    mutate_and_reject("VOP no-match warning suppressed", corpus, schema, |v| {
        v["scenarios"][30]["payer_warning_state"] = json!("not_required");
    })?;
    count += 1;

    mutate_and_reject(
        "VOP unavailable result treated as fraud mismatch",
        corpus,
        schema,
        |v| {
            v["scenarios"][31]["payee_binding"] = json!("mismatch");
            v["scenarios"][31]["expected_disposition"] = json!("reject_payee_mismatch");
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
                    "sa-payment-rtp-abuse-evaluator [--corpus PATH] [--schema PATH] [--self-test]"
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
        "evaluator": "mycelix.sa-payment-rtp-abuse-evaluator.v1",
        "status": status,
        "external_effects_enabled": false,
        "claim_ceiling": "synthetic requester/authenticity/authority/commercial-claim/accessibility/privacy checks only",
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
        assert_eq!(array_at(&corpus, "source_register").unwrap().len(), 6);
        assert_eq!(array_at(&corpus, "profiles").unwrap().len(), 3);
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 32);
    }

    #[test]
    fn negative_mutation_campaign_rejects_claim_laundering_and_abuse() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self tests"), 28);
    }

    #[test]
    fn vop_name_match_never_substitutes_for_collection_authority() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let matched = &corpus["scenarios"][28];
        assert_eq!(str_at(matched, "payee_verification_result"), Some("match"));
        assert_eq!(
            str_at(matched, "collection_authority"),
            Some("not_provided")
        );
        assert_eq!(
            str_at(matched, "expected_claim_ceiling"),
            Some("account_name_match_only")
        );

        let no_match = &corpus["scenarios"][30];
        assert_eq!(
            str_at(no_match, "payer_warning_state"),
            Some("warning_displayed")
        );
        assert_eq!(str_at(no_match, "payer_choice_state"), Some("preserved"));
    }

    #[test]
    fn account_name_match_never_equals_legal_identity_or_collect_authority() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let scenario = &corpus["scenarios"][1];
        assert_eq!(
            str_at(scenario, "expected_claim_ceiling"),
            Some("account_name_match_only")
        );
        assert_ne!(
            str_at(scenario, "collection_authority"),
            Some("verified_for_beneficiary")
        );
    }
}
