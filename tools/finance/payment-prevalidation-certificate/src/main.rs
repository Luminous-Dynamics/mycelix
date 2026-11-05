#![forbid(unsafe_code)]

//! Effects-disabled compositional payment pre-initiation certificate evaluator.
//! This consumes upstream typed evidence; it does not implement QR, VoP, JOSE, directory
//! lookup, payment dispatch, settlement or finality.

use jsonschema::draft202012;
use serde_json::{Map, Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const CORPUS: &str = include_str!(
    "../../../../mycelix-finance/docs/payment-prevalidation-certificate-corpus-v1.json"
);
const SCHEMA: &str = include_str!(
    "../../../../mycelix-finance/docs/payment-prevalidation-certificate-v1.schema.json"
);

const PRESENTATION_DOMAIN: &str = "MYCELIX-PAYER-PRESENTATION-V1";
const EVIDENCE_SET_DOMAIN: &str = "MYCELIX-EVIDENCE-SET-V1";
const CERTIFICATE_DOMAIN: &str = "MYCELIX-PREINITIATION-CERTIFICATE-V1";

#[derive(Clone, Debug)]
struct CaseDecision {
    admission: &'static str,
    disposition: &'static str,
    reason: &'static str,
    certificate_digest: String,
}

fn str_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn u64_at(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn arr_at<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn as_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

fn digest_domain(domain: &str, value: &Value) -> Result<String, String> {
    // serde_json's default Map is key-ordered (BTreeMap); no preserve_order feature
    // is enabled for this crate. Arrays retain their source order.
    let canonical = serde_json::to_vec(value).map_err(|error| error.to_string())?;
    let mut hasher = Sha256::new();
    hasher.update(domain.as_bytes());
    hasher.update([0_u8]);
    hasher.update(canonical);
    Ok(format!("sha256:{}", as_hex(&hasher.finalize())))
}

fn days_from_civil(year: i64, month: i64, day: i64) -> i64 {
    let adjusted_year = year - if month <= 2 { 1 } else { 0 };
    let era = adjusted_year.div_euclid(400);
    let year_of_era = adjusted_year - era * 400;
    let adjusted_month = month + if month > 2 { -3 } else { 9 };
    let day_of_year = (153 * adjusted_month + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
}

fn parse_utc_seconds(value: &str) -> Option<i64> {
    if value.len() != 20 || !value.ends_with('Z') || value.as_bytes().get(10) != Some(&b'T') {
        return None;
    }
    let year = value.get(0..4)?.parse::<i64>().ok()?;
    let month = value.get(5..7)?.parse::<i64>().ok()?;
    let day = value.get(8..10)?.parse::<i64>().ok()?;
    let hour = value.get(11..13)?.parse::<i64>().ok()?;
    let minute = value.get(14..16)?.parse::<i64>().ok()?;
    let second = value.get(17..19)?.parse::<i64>().ok()?;
    if year < 1970
        || !(1..=12).contains(&month)
        || !(1..=31).contains(&day)
        || !(0..=23).contains(&hour)
        || !(0..=59).contains(&minute)
        || !(0..=59).contains(&second)
    {
        return None;
    }
    Some(days_from_civil(year, month, day) * 86_400 + hour * 3_600 + minute * 60 + second)
}

fn evidence_fresh_at(
    observed_at: &Value,
    expires_at: &Value,
    evaluation_time: &str,
    maximum_age_seconds: u64,
) -> bool {
    let Some(observed) = observed_at.as_str().and_then(parse_utc_seconds) else {
        return false;
    };
    let Some(expires) = expires_at.as_str().and_then(parse_utc_seconds) else {
        return false;
    };
    let Some(evaluation) = parse_utc_seconds(evaluation_time) else {
        return false;
    };
    observed <= evaluation
        && evaluation < expires
        && evaluation - observed <= maximum_age_seconds.min(i64::MAX as u64) as i64
}

fn digest_is_well_formed(value: &str) -> bool {
    value.strip_prefix("sha256:").is_some_and(|hex| {
        hex.len() == 64
            && hex
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    })
}

fn contains_string(values: &[Value], expected: &str) -> bool {
    values.iter().any(|value| value.as_str() == Some(expected))
}

fn source_exists(sources: &BTreeSet<String>, reference: &Value) -> bool {
    reference
        .as_str()
        .is_some_and(|reference| sources.contains(reference))
}

fn admission(
    scenario: &Value,
    profile: Option<&Value>,
    source_ids: &BTreeSet<String>,
    required_fields: &[Value],
) -> CaseDecision {
    let scenario_id = str_at(scenario, "scenario_id").unwrap_or("UNKNOWN");
    let intent = &scenario["intent"];
    let payee = &scenario["payee_resolution"];
    let vop = &scenario["vop_result"];
    let provider = &scenario["provider_evidence"];
    let limit = &scenario["limit_fee"];
    let presentation = &scenario["presentation"];
    let consent = &scenario["consent"];

    let decide = |state: &'static str, disposition: &'static str, reason: &'static str| {
        let payload = json!({
            "scenario_id": scenario["scenario_id"],
            "profile_id": scenario["profile_id"],
            "intent_digest": scenario["intent"]["intent_digest"],
            "admission": state,
            "disposition": disposition,
            "reason": reason
        });
        let certificate_digest = digest_domain(CERTIFICATE_DOMAIN, &payload).unwrap_or_default();
        CaseDecision {
            admission: state,
            disposition,
            reason,
            certificate_digest,
        }
    };

    let Some(profile) = profile else {
        return decide(
            "quarantined_unknown",
            "quarantined_unknown",
            "participant_profile_unresolved",
        );
    };

    if scenario
        .get("profile_expired_override")
        .and_then(Value::as_bool)
        == Some(true)
    {
        return decide(
            "quarantined_unknown",
            "quarantined_unknown",
            "profile_expired_or_not_current",
        );
    }

    let profile_id = str_at(profile, "profile_id").unwrap_or("");
    let evaluation_time = str_at(scenario, "evaluation_time").unwrap_or("");
    let evaluation_date = evaluation_time.get(0..10).unwrap_or("");
    let profile_observed = str_at(profile, "observed_on").unwrap_or("");
    let valid_from = profile.get("valid_from").and_then(Value::as_str);
    let effective_to = profile.get("effective_to").and_then(Value::as_str);
    if profile_observed > evaluation_date
        || valid_from.is_some_and(|start| start > evaluation_date)
        || effective_to.is_some_and(|end| evaluation_date > end)
    {
        return decide(
            "quarantined_unknown",
            "quarantined_unknown",
            "participant_profile_not_valid_at_evaluation_time",
        );
    }
    let scenario_profile = str_at(scenario, "profile_id").unwrap_or("");
    if profile_id != scenario_profile
        || str_at(intent, "profile_id") != Some(profile_id)
        || str_at(payee, "profile_id") != Some(profile_id)
        || str_at(vop, "profile_id") != Some(profile_id)
        || str_at(limit, "profile_id") != Some(profile_id)
    {
        return decide(
            "rejected_evidence",
            "rejected_evidence",
            "profile_binding_mismatch",
        );
    }

    evaluate_bound_case(scenario, profile, source_ids, required_fields, decide)
}

fn evaluate_bound_case<F>(
    scenario: &Value,
    profile: &Value,
    source_ids: &BTreeSet<String>,
    required_fields: &[Value],
    decide: F,
) -> CaseDecision
where
    F: Fn(&'static str, &'static str, &'static str) -> CaseDecision,
{
    let intent = &scenario["intent"];
    let payee = &scenario["payee_resolution"];
    let vop = &scenario["vop_result"];
    let provider = &scenario["provider_evidence"];
    let limit = &scenario["limit_fee"];
    let presentation = &scenario["presentation"];
    let consent = &scenario["consent"];
    let profile_id = str_at(profile, "profile_id").unwrap_or("");
    let intent_digest = str_at(intent, "intent_digest").unwrap_or("");
    let beneficiary = str_at(intent, "beneficiary_subject_commitment").unwrap_or("");
    let vop_status = str_at(vop, "status").unwrap_or("unknown");
    let evaluation_time = str_at(scenario, "evaluation_time").unwrap_or("");
    if !evidence_fresh_at(
        &intent["issued_at"],
        &intent["expires_at"],
        evaluation_time,
        u64::MAX,
    ) {
        return decide(
            "quarantined_unknown",
            "quarantined_unknown",
            "payment_intent_not_current_at_certificate_evaluation",
        );
    }

    if str_at(intent, "digest_state") != Some("verified_by_upstream_intent_contract")
        || !digest_is_well_formed(intent_digest)
        || str_at(intent, "digest_evidence_ref").is_none()
    {
        return decide(
            "rejected_evidence",
            "rejected_evidence",
            "upstream_intent_digest_not_verified",
        );
    }

    if str_at(intent, "jurisdiction") != str_at(profile, "jurisdiction")
        || str_at(intent, "payer_participant") != str_at(profile, "payer_participant")
        || str_at(intent, "payee_participant") != str_at(profile, "payee_participant")
        || str_at(intent, "currency") != str_at(profile, "currency")
        || str_at(intent, "instrument") != str_at(profile, "instrument")
    {
        return decide(
            "quarantined_unknown",
            "quarantined_unknown",
            "intent_not_admissible_under_participant_profile",
        );
    }

    if !source_exists(source_ids, &profile["limit_fee_source_ref"])
        || !arr_at(profile, "source_refs").is_some_and(|refs| {
            !refs.is_empty()
                && refs
                    .iter()
                    .all(|reference| source_exists(source_ids, reference))
        })
        || !source_exists(source_ids, &payee["source_ref"])
        || !source_exists(source_ids, &limit["source_ref"])
    {
        return decide(
            "quarantined_unknown",
            "quarantined_unknown",
            "required_source_reference_unresolved",
        );
    }

    let payee_freshness = u64_at(profile, "payee_resolution_freshness_seconds").unwrap_or(0);
    if payee_freshness == 0
        || str_at(payee, "status") == Some("stale")
        || str_at(payee, "status") == Some("unknown")
        || str_at(payee, "freshness_state") != Some("fresh")
        || !evidence_fresh_at(
            &payee["observed_at"],
            &payee["expires_at"],
            evaluation_time,
            payee_freshness,
        )
    {
        return decide(
            "quarantined_unknown",
            "quarantined_unknown",
            "payee_resolution_stale_or_indeterminate",
        );
    }
    if str_at(payee, "status") != Some("resolved")
        || str_at(payee, "provider_authentication_state") != Some("authenticated_fixture")
        || str_at(payee, "provider") != str_at(profile, "payee_participant")
        || str_at(payee, "participant") != str_at(profile, "payee_participant")
        || str_at(payee, "subject_commitment") != Some(beneficiary)
        || str_at(payee, "intent_digest") != Some(intent_digest)
    {
        return decide(
            "rejected_evidence",
            "rejected_evidence",
            "payee_resolution_subject_provider_or_intent_mismatch",
        );
    }

    if str_at(profile, "vop_mode") == Some("unknown") {
        return decide(
            "quarantined_unknown",
            "quarantined_unknown",
            "jurisdictional_vop_capability_unqualified",
        );
    }

    let warning_required = matches!(
        vop_status,
        "close_match" | "no_match" | "unavailable" | "not_supported"
    );

    match str_at(profile, "vop_mode") {
        Some("proposal_only") => {
            if vop_status != "not_supported"
                || !provider.is_null()
                || str_at(profile, "vop_authority").is_some()
            {
                return decide(
                    "rejected_evidence",
                    "rejected_evidence",
                    "positive_vop_claim_for_proposal_only_profile",
                );
            }
            if !source_exists(source_ids, &vop["source_ref"]) {
                return decide(
                    "quarantined_unknown",
                    "quarantined_unknown",
                    "proposal_only_source_unresolved",
                );
            }
        }
        Some("supported") => {
            let vop_freshness = u64_at(profile, "vop_freshness_seconds").unwrap_or(0);
            if vop_freshness == 0
                || !evidence_fresh_at(
                    &vop["observed_at"],
                    &vop["expires_at"],
                    evaluation_time,
                    vop_freshness,
                )
            {
                return decide(
                    "quarantined_unknown",
                    "quarantined_unknown",
                    "vop_result_timestamp_expired_future_or_over_age",
                );
            }
            if str_at(vop, "scheme") != str_at(profile, "vop_scheme")
                || str_at(vop, "profile_id") != Some(profile_id)
                || str_at(vop, "subject_commitment") != Some(beneficiary)
                || str_at(vop, "intent_digest") != Some(intent_digest)
                || !source_exists(source_ids, &vop["source_ref"])
            {
                return decide(
                    "rejected_evidence",
                    "rejected_evidence",
                    "vop_result_subject_profile_or_source_mismatch",
                );
            }
            if !matches!(
                vop_status,
                "match" | "close_match" | "no_match" | "unavailable"
            ) {
                return decide(
                    "quarantined_unknown",
                    "quarantined_unknown",
                    "vop_result_class_unknown",
                );
            }
            if provider.is_null() {
                return decide(
                    "quarantined_unknown",
                    "quarantined_unknown",
                    "provider_result_has_no_verification_evidence",
                );
            }
            if str_at(provider, "verification_state") == Some("indeterminate")
                || str_at(provider, "key_trust_state") == Some("unknown")
                || str_at(provider, "freshness_state") != Some("fresh")
                || str_at(provider, "algorithm_policy_state") == Some("unknown")
                || str_at(provider, "replay_state") == Some("unknown")
            {
                return decide(
                    "quarantined_unknown",
                    "quarantined_unknown",
                    "provider_evidence_trust_or_freshness_indeterminate",
                );
            }
            if !evidence_fresh_at(
                &provider["issued_at"],
                &provider["expires_at"],
                evaluation_time,
                vop_freshness,
            ) {
                return decide(
                    "quarantined_unknown",
                    "quarantined_unknown",
                    "provider_signed_evidence_timestamp_expired_future_or_over_age",
                );
            }
            if str_at(provider, "verification_state") != Some("verified_fixture")
                || str_at(provider, "issuer") != str_at(profile, "vop_authority")
                || str_at(provider, "audience") != str_at(profile, "vop_audience")
                || str_at(provider, "response_type") != Some("payee-verification-result")
                || str_at(provider, "profile_id") != Some(profile_id)
                || str_at(provider, "subject_commitment") != Some(beneficiary)
                || str_at(provider, "intent_digest") != Some(intent_digest)
                || str_at(provider, "algorithm_policy_state") != Some("allowed")
                || str_at(provider, "key_trust_state") != Some("authorised_current")
                || str_at(provider, "key_source_state") != Some("profile_allowlisted")
                || str_at(provider, "replay_state") != Some("not_replayed")
                || !source_exists(source_ids, &provider["source_ref"])
            {
                return decide(
                    "rejected_evidence",
                    "rejected_evidence",
                    "provider_signature_issuer_audience_key_subject_or_replay_invalid",
                );
            }
        }
        _ => {
            return decide(
                "quarantined_unknown",
                "quarantined_unknown",
                "vop_profile_mode_unknown",
            );
        }
    }

    if let Some(risk) = scenario.get("risk_signal")
        && str_at(risk, "state") == Some("stale")
        && risk.get("used_for_admission").and_then(Value::as_bool) == Some(true)
    {
        return decide(
            "quarantined_unknown",
            "quarantined_unknown",
            "stale_risk_signal_used_as_admission_authority",
        );
    }

    if let Some(transport) = scenario.get("transport_payload") {
        if u64_at(transport, "amount_minor_units") != u64_at(intent, "amount_minor_units")
            || str_at(transport, "currency") != str_at(intent, "currency")
            || str_at(transport, "intent_digest") != Some(intent_digest)
        {
            return decide(
                "rejected_evidence",
                "rejected_evidence",
                "transport_payload_and_intent_disagree",
            );
        }
    }

    let limit_fee_freshness = u64_at(profile, "limit_fee_freshness_seconds").unwrap_or(0);
    if limit_fee_freshness == 0
        || !evidence_fresh_at(
            &limit["observed_at"],
            &limit["expires_at"],
            evaluation_time,
            limit_fee_freshness,
        )
    {
        return decide(
            "quarantined_unknown",
            "quarantined_unknown",
            "participant_limit_or_fee_evidence_timestamp_expired_future_or_over_age",
        );
    }

    match str_at(limit, "status") {
        Some("unknown") | None => {
            return decide(
                "quarantined_unknown",
                "quarantined_unknown",
                "participant_limit_or_fee_state_unknown",
            );
        }
        Some("denied") => {
            return decide(
                "ineligible_under_profile",
                "ineligible_under_profile",
                "participant_profile_limit_rejected_intent",
            );
        }
        Some("allowed") => {}
        _ => {
            return decide(
                "quarantined_unknown",
                "quarantined_unknown",
                "participant_limit_state_unrecognised",
            );
        }
    }

    if str_at(limit, "participant") != str_at(profile, "payer_participant")
        || u64_at(limit, "amount_minor_units") != u64_at(intent, "amount_minor_units")
        || str_at(limit, "currency") != str_at(intent, "currency")
    {
        return decide(
            "quarantined_unknown",
            "quarantined_unknown",
            "limit_evidence_not_bound_to_exact_intent",
        );
    }

    if limit.get("fee_disclosed").and_then(Value::as_bool) != Some(true)
        || u64_at(limit, "fee_minor_units").is_none()
        || matches!(str_at(limit, "fee_payer"), Some("unknown") | None)
    {
        return decide(
            "rejected_evidence",
            "rejected_evidence",
            "fee_not_disclosed_or_fee_incidence_unknown",
        );
    }

    let required_missing = required_fields
        .iter()
        .filter_map(Value::as_str)
        .any(|field| {
            !arr_at(presentation, "fields_shown")
                .is_some_and(|fields| contains_string(fields, field))
        });
    if required_missing {
        return decide(
            "rejected_evidence",
            "rejected_evidence",
            "payer_presentation_omits_material_field",
        );
    }

    let payload = &presentation["payload"];
    let presentation_digest = match digest_domain(PRESENTATION_DOMAIN, payload) {
        Ok(value) => value,
        Err(_) => {
            return decide(
                "rejected_evidence",
                "rejected_evidence",
                "presentation_canonicalisation_failed",
            );
        }
    };

    if str_at(presentation, "signed_payload") != Some(presentation_digest.as_str()) {
        return decide(
            "rejected_evidence",
            "rejected_evidence",
            "payer_presentation_digest_does_not_recompute",
        );
    }

    if str_at(payload, "profile_id") != Some(profile_id)
        || str_at(payload, "intent_digest") != Some(intent_digest)
        || str_at(payload, "beneficiary_subject_commitment") != Some(beneficiary)
        || u64_at(payload, "amount_minor_units") != u64_at(intent, "amount_minor_units")
        || str_at(payload, "currency") != str_at(intent, "currency")
        || str_at(payload, "instrument") != str_at(intent, "instrument")
        || str_at(payload, "purpose") != str_at(intent, "purpose")
        || str_at(payload, "reference") != str_at(intent, "reference")
        || u64_at(payload, "fee_minor_units") != u64_at(limit, "fee_minor_units")
        || str_at(payload, "fee_payer") != str_at(limit, "fee_payer")
        || str_at(payload, "participant") != str_at(profile, "payer_participant")
        || str_at(payload, "rail") != str_at(intent, "rail")
        || str_at(payload, "channel") != str_at(intent, "channel")
        || str_at(payload, "expiry_at") != str_at(intent, "expires_at")
        || str_at(payload, "verification_result") != Some(vop_status)
        || arr_at(presentation, "warnings_shown") != arr_at(payload, "warnings")
    {
        return decide(
            "needs_payer_reconfirmation",
            "needs_payer_reconfirmation",
            "payer_presentation_no_longer_matches_exact_intent_or_evidence",
        );
    }

    if warning_required
        && arr_at(presentation, "warnings_shown").is_none_or(|warnings| warnings.is_empty())
    {
        return decide(
            "rejected_evidence",
            "rejected_evidence",
            "verification_warning_not_presented_to_payer",
        );
    }

    let policy = match vop_status {
        "close_match" | "no_match" => {
            str_at(profile, "payer_override_no_match").unwrap_or("unknown")
        }
        "unavailable" => str_at(profile, "payer_override_unavailable").unwrap_or("unknown"),
        "not_supported" => "allowed_with_warning",
        "match" => "allowed",
        _ => "unknown",
    };

    let base_admission = match policy {
        "allowed" => "ready_for_payer_decision",
        "allowed_with_warning" => "ready_with_warning",
        "not_allowed" => {
            return decide(
                "ineligible_under_profile",
                "ineligible_under_profile",
                "profile_does_not_allow_payer_override",
            );
        }
        "unknown" => {
            return decide(
                "quarantined_unknown",
                "quarantined_unknown",
                "payer_override_policy_unknown",
            );
        }
        _ => {
            return decide(
                "quarantined_unknown",
                "quarantined_unknown",
                "payer_override_policy_unrecognised",
            );
        }
    };

    let evidence_digest = match digest_domain(
        EVIDENCE_SET_DOMAIN,
        &json!({
            "profile_id": profile_id,
            "intent_digest": intent_digest,
            "evidence_refs": scenario["evidence_refs"]
        }),
    ) {
        Ok(value) => value,
        Err(_) => {
            return decide(
                "rejected_evidence",
                "rejected_evidence",
                "evidence_set_canonicalisation_failed",
            );
        }
    };

    if str_at(scenario, "evidence_set_digest") != Some(evidence_digest.as_str()) {
        return decide(
            "rejected_evidence",
            "rejected_evidence",
            "scenario_evidence_set_digest_does_not_recompute",
        );
    }

    let consent_decision = str_at(consent, "decision").unwrap_or("unknown");
    if !matches!(consent_decision, "not_yet_decided") {
        let decision_at = consent["observed_at"]
            .as_str()
            .and_then(parse_utc_seconds);
        let presentation_at = presentation["presented_at"]
            .as_str()
            .and_then(parse_utc_seconds);
        let evaluation_at = parse_utc_seconds(evaluation_time);
        let intent_expiry = intent["expires_at"]
            .as_str()
            .and_then(parse_utc_seconds);
        if decision_at.is_none()
            || presentation_at.is_none()
            || evaluation_at.is_none()
            || intent_expiry.is_none()
            || decision_at < presentation_at
            || decision_at > evaluation_at
            || decision_at >= intent_expiry
        {
            return decide(
                "needs_payer_reconfirmation",
                "needs_payer_reconfirmation",
                "payer_decision_not_after_presentation_and_before_evaluation_or_expiry",
            );
        }
    }
    let disposition = match consent_decision {
        "not_yet_decided" => {
            if str_at(consent, "authority_state") != Some("not_applicable") {
                return decide(
                    "rejected_evidence",
                    "rejected_evidence",
                    "non_decision_has_unexpected_authority_state",
                );
            }
            if str_at(consent, "presentation_digest") != Some(presentation_digest.as_str()) {
                return decide(
                    "needs_payer_reconfirmation",
                    "needs_payer_reconfirmation",
                    "presentation_changed_before_payer_decision",
                );
            }
            "awaiting_payer_decision"
        }
        "declined" => {
            if str_at(consent, "authority_state") != Some("valid_fixture") {
                return decide(
                    "rejected_evidence",
                    "rejected_evidence",
                    "decline_event_lacks_valid_payer_authority",
                );
            }
            "payer_declined"
        }
        "query" => {
            if str_at(consent, "authority_state") != Some("valid_fixture") {
                return decide(
                    "rejected_evidence",
                    "rejected_evidence",
                    "payer_query_lacks_valid_authority",
                );
            }
            "awaiting_payer_response"
        }
        "reconfirm" => "needs_payer_reconfirmation",
        "accepted" => {
            if str_at(consent, "authority_state") != Some("valid_fixture")
                || str_at(consent, "nonce").is_none()
            {
                return decide(
                    "rejected_evidence",
                    "rejected_evidence",
                    "payer_acceptance_authority_or_nonce_invalid",
                );
            }
            if str_at(consent, "intent_digest") != Some(intent_digest)
                || str_at(consent, "presentation_digest") != Some(presentation_digest.as_str())
                || str_at(consent, "evidence_set_digest") != Some(evidence_digest.as_str())
            {
                return decide(
                    "needs_payer_reconfirmation",
                    "needs_payer_reconfirmation",
                    "payer_consent_digest_binding_is_stale",
                );
            }
            let accepted_at = consent["observed_at"].as_str().and_then(parse_utc_seconds);
            let presented_at = presentation["presented_at"]
                .as_str()
                .and_then(parse_utc_seconds);
            let evaluation_at = parse_utc_seconds(evaluation_time);
            let expires_at = intent["expires_at"].as_str().and_then(parse_utc_seconds);
            if accepted_at.is_none()
                || presented_at.is_none()
                || evaluation_at.is_none()
                || expires_at.is_none()
                || accepted_at <= presented_at
                || accepted_at > evaluation_at
                || accepted_at >= expires_at
            {
                return decide(
                    "needs_payer_reconfirmation",
                    "needs_payer_reconfirmation",
                    "payer_consent_time_outside_presentation_or_intent_window",
                );
            }
            "prevalidated_for_psp_review"
        }
        _ => {
            return decide(
                "rejected_evidence",
                "rejected_evidence",
                "unknown_payer_decision",
            );
        }
    };

    let admission_state = if disposition == "needs_payer_reconfirmation" {
        "needs_payer_reconfirmation"
    } else {
        base_admission
    };

    let certificate_payload = json!({
        "scenario_id": scenario["scenario_id"],
        "evaluation_time": evaluation_time,
        "profile_id": profile_id,
        "intent_digest": intent_digest,
        "payee_resolution_ref": payee["evidence_ref"],
        "vop_status": vop_status,
        "provider_evidence_ref": provider["evidence_ref"],
        "limit_fee_evidence_ref": limit["evidence_ref"],
        "presentation_digest": presentation_digest,
        "evidence_set_digest": evidence_digest,
        "payer_decision": consent_decision,
        "admission": admission_state,
        "disposition": disposition
    });
    let certificate_digest =
        digest_domain(CERTIFICATE_DOMAIN, &certificate_payload).unwrap_or_default();

    CaseDecision {
        admission: admission_state,
        disposition,
        reason: "all_composed_preinitiation_gates_satisfied",
        certificate_digest,
    }
}

fn schema_errors(schema: &Value, corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if !jsonschema::meta::is_valid(schema) {
        errors.push("schema is invalid against its Draft 2020-12 meta-schema".into());
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

fn semantic_evaluation(corpus: &Value) -> (Vec<String>, Vec<Value>) {
    let mut errors = Vec::new();
    let mut results = Vec::new();

    if corpus
        .get("external_effects_enabled")
        .and_then(Value::as_bool)
        != Some(false)
        || corpus.get("dispatch_enabled").and_then(Value::as_bool) != Some(false)
    {
        errors.push("external effects and dispatch must remain disabled".into());
    }

    let Some(as_of) = str_at(corpus, "as_of") else {
        errors.push("corpus as_of date is missing".into());
        return (errors, results);
    };
    if as_of != "2026-11-05" {
        errors.push("the frozen corpus as_of date changed without a version update".into());
    }

    let sources = arr_at(corpus, "source_register")
        .cloned()
        .unwrap_or_default();
    let profiles = arr_at(corpus, "profiles").cloned().unwrap_or_default();
    let scenarios = arr_at(corpus, "scenarios").cloned().unwrap_or_default();
    let required_fields = arr_at(corpus, "required_presentation_fields")
        .cloned()
        .unwrap_or_default();

    let mut source_ids = BTreeSet::new();
    for (index, source) in sources.iter().enumerate() {
        let id = str_at(source, "id").unwrap_or("");
        if id.is_empty() || !source_ids.insert(id.to_owned()) {
            errors.push(format!(
                "source_register[{index}]: missing or duplicate source id"
            ));
        }
    }

    let mut profile_map = BTreeMap::new();
    for (index, profile) in profiles.iter().enumerate() {
        let id = str_at(profile, "profile_id").unwrap_or("");
        if id.is_empty() || profile_map.insert(id.to_owned(), profile).is_some() {
            errors.push(format!(
                "profiles[{index}]: missing or duplicate profile id"
            ));
        }
        for reference in arr_at(profile, "source_refs").cloned().unwrap_or_default() {
            if !source_exists(&source_ids, &reference) {
                errors.push(format!("profiles[{index}]: unresolved source reference"));
            }
        }
        if !source_exists(&source_ids, &profile["limit_fee_source_ref"]) {
            errors.push(format!("profiles[{index}]: unresolved limit/fee source"));
        }
    }

    let mut scenario_ids = BTreeSet::new();
    for (index, scenario) in scenarios.iter().enumerate() {
        let id = str_at(scenario, "scenario_id").unwrap_or("");
        if id.is_empty() || !scenario_ids.insert(id.to_owned()) {
            errors.push(format!(
                "scenarios[{index}]: missing or duplicate scenario id"
            ));
            continue;
        }

        let profile_id = str_at(scenario, "profile_id").unwrap_or("");
        let profile = profile_map.get(profile_id).copied();
        let result = admission(scenario, profile, &source_ids, &required_fields);

        if result.admission != str_at(scenario, "expected_admission").unwrap_or("")
            || result.disposition != str_at(scenario, "expected_disposition").unwrap_or("")
        {
            errors.push(format!(
                "scenarios[{index}] {id}: computed admission/disposition differs from frozen expected result ({} / {})",
                result.admission, result.disposition
            ));
        }
        if scenario.get("dispatch_allowed").and_then(Value::as_bool) != Some(false) {
            errors.push(format!(
                "scenarios[{index}] {id}: certificate corpus must never allow dispatch"
            ));
        }

        results.push(json!({
            "scenario_id": id,
            "admission": result.admission,
            "disposition": result.disposition,
            "reason": result.reason,
            "certificate_digest": result.certificate_digest,
            "dispatch_allowed": false
        }));
    }

    (errors, results)
}

fn evaluate_values(corpus: &Value, schema: &Value) -> (Vec<String>, Vec<Value>) {
    let (mut errors, results) = semantic_evaluation(corpus);
    errors.extend(schema_errors(schema, corpus));
    (errors, results)
}

fn mutate_and_reject(
    label: &str,
    corpus: &Value,
    schema: &Value,
    mutation: impl FnOnce(&mut Value),
) -> Result<(), String> {
    let mut changed = corpus.clone();
    mutation(&mut changed);
    let (errors, _) = evaluate_values(&changed, schema);
    if errors.is_empty() {
        return Err(format!("negative control was not rejected: {label}"));
    }
    Ok(())
}

fn run_self_tests(corpus: &Value, schema: &Value) -> Result<usize, String> {
    let (baseline, _) = evaluate_values(corpus, schema);
    if !baseline.is_empty() {
        return Err(format!(
            "baseline must pass before mutation campaign: {baseline:?}"
        ));
    }
    let mut count = 0usize;

    mutate_and_reject("external effects enabled", corpus, schema, |value| {
        value["external_effects_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject("dispatch enabled", corpus, schema, |value| {
        value["dispatch_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject("payee subject substituted", corpus, schema, |value| {
        value["scenarios"][0]["payee_resolution"]["subject_commitment"] = json!("subject:attacker");
    })?;
    count += 1;

    mutate_and_reject("VoP result cross-subject replay", corpus, schema, |value| {
        value["scenarios"][0]["vop_result"]["subject_commitment"] = json!("subject:other");
    })?;
    count += 1;

    mutate_and_reject("wrong provider issuer", corpus, schema, |value| {
        value["scenarios"][0]["provider_evidence"]["issuer"] = json!("attacker");
    })?;
    count += 1;

    mutate_and_reject("wrong provider audience", corpus, schema, |value| {
        value["scenarios"][0]["provider_evidence"]["audience"] = json!("wrong-audience");
    })?;
    count += 1;

    mutate_and_reject("provider response replay", corpus, schema, |value| {
        value["scenarios"][0]["provider_evidence"]["replay_state"] = json!("replayed");
    })?;
    count += 1;

    mutate_and_reject(
        "provider untrusted remote key source",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["provider_evidence"]["key_source_state"] =
                json!("untrusted_remote_url");
        },
    )?;
    count += 1;

    mutate_and_reject("stale payee resolution", corpus, schema, |value| {
        value["scenarios"][0]["payee_resolution"]["freshness_state"] = json!("stale");
    })?;
    count += 1;

    mutate_and_reject("stale presentation commitment", corpus, schema, |value| {
        value["scenarios"][0]["presentation"]["payload"]["amount_minor_units"] = json!(2000);
    })?;
    count += 1;

    mutate_and_reject("fee not disclosed", corpus, schema, |value| {
        value["scenarios"][0]["limit_fee"]["fee_disclosed"] = json!(false);
    })?;
    count += 1;

    mutate_and_reject("consent bound to another intent", corpus, schema, |value| {
        value["scenarios"][3]["consent"]["intent_digest"] =
            json!("sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
    })?;
    count += 1;

    mutate_and_reject("consent evidence-set replay", corpus, schema, |value| {
        value["scenarios"][3]["consent"]["evidence_set_digest"] =
            json!("sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc");
    })?;
    count += 1;

    mutate_and_reject(
        "positive VoP invented for proposal-only South Africa",
        corpus,
        schema,
        |value| {
            value["scenarios"][28]["vop_result"]["status"] = json!("match");
        },
    )?;
    count += 1;

    mutate_and_reject("unknown profile promoted", corpus, schema, |value| {
        value["scenarios"][20]["profile_id"] = json!("EU-EPC-VOP-1.1-V1");
    })?;
    count += 1;

    mutate_and_reject("limit bypass", corpus, schema, |value| {
        value["scenarios"][22]["limit_fee"]["status"] = json!("allowed");
        value["scenarios"][22]["expected_admission"] = json!("ready_for_payer_decision");
        value["scenarios"][22]["expected_disposition"] = json!("awaiting_payer_decision");
    })?;
    count += 1;

    mutate_and_reject(
        "expired provider evidence despite fresh label",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["provider_evidence"]["expires_at"] =
                json!("2026-11-05T09:02:59Z");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "expired VoP result despite fresh label",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["vop_result"]["expires_at"] = json!("2026-11-05T09:02:59Z");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "expired payee resolution despite fresh label",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["payee_resolution"]["expires_at"] =
                json!("2026-11-05T09:02:59Z");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "future-dated provider evidence despite fresh label",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["provider_evidence"]["issued_at"] =
                json!("2026-11-05T09:03:01Z");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "stale fee evidence despite allowed status",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["limit_fee"]["observed_at"] =
                json!("2026-11-05T08:52:59Z");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "payer decision after evaluation time",
        corpus,
        schema,
        |value| {
            value["scenarios"][3]["consent"]["observed_at"] = json!("2026-11-05T09:03:01Z");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "payer decision predates displayed presentation",
        corpus,
        schema,
        |value| {
            value["scenarios"][2]["consent"]["observed_at"] = json!("2026-11-05T09:01:40Z");
        },
    )?;
    count += 1;

    mutate_and_reject(
        "profile effective date begins after evaluation",
        corpus,
        schema,
        |value| {
            value["profiles"][0]["valid_from"] = json!("2026-11-06");
        },
    )?;
    count += 1;

    Ok(count)
}

fn run(corpus_text: &str, schema_text: &str, self_test: bool) -> Result<Value, String> {
    let corpus: Value =
        serde_json::from_str(corpus_text).map_err(|error| format!("corpus JSON: {error}"))?;
    let schema: Value =
        serde_json::from_str(schema_text).map_err(|error| format!("schema JSON: {error}"))?;
    let (errors, results) = evaluate_values(&corpus, &schema);
    let mutation_count = if self_test {
        run_self_tests(&corpus, &schema)?
    } else {
        0
    };
    let mut admission_counts = BTreeMap::<String, usize>::new();
    let mut disposition_counts = BTreeMap::<String, usize>::new();
    for result in &results {
        if let Some(value) = str_at(result, "admission") {
            *admission_counts.entry(value.to_owned()).or_default() += 1;
        }
        if let Some(value) = str_at(result, "disposition") {
            *disposition_counts.entry(value.to_owned()).or_default() += 1;
        }
    }
    Ok(json!({
        "evaluator": "mycelix.payment-prevalidation-certificate.v1",
        "status": if errors.is_empty() { "PASS" } else { "FAIL" },
        "external_effects_enabled": false,
        "dispatch_enabled": false,
        "claim_ceiling": "synthetic pre-initiation evidence composition only",
        "scenario_count": results.len(),
        "admission_counts": admission_counts,
        "disposition_counts": disposition_counts,
        "negative_mutations": mutation_count,
        "certificates": results,
        "errors": errors
    }))
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut corpus_text = CORPUS.to_owned();
    let mut schema_text = SCHEMA.to_owned();
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
                    "payment-prevalidation-certificate [--corpus PATH] [--schema PATH] [--self-test]"
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

    let report = run(&corpus_text, &schema_text, self_test)
        .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    if report["status"] != "PASS" {
        std::process::exit(1);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checked_in_corpus_schema_and_composed_certificates_pass() {
        let report = run(CORPUS, SCHEMA, false).expect("evaluation");
        assert_eq!(report["status"], "PASS", "{}", report["errors"]);
        assert_eq!(report["scenario_count"], 36);
        assert_eq!(report["dispatch_enabled"], false);
        assert!(
            report["certificates"]
                .as_array()
                .unwrap()
                .iter()
                .all(|item| item["certificate_digest"]
                    .as_str()
                    .unwrap()
                    .starts_with("sha256:"))
        );
    }

    #[test]
    fn deterministic_negative_mutation_campaign_rejects_composition_failures() {
        let corpus: Value = serde_json::from_str(CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("mutations"), 24);
    }

    #[test]
    fn utc_timestamp_freshness_is_checked_against_evaluation_time() {
        assert!(evidence_fresh_at(
            &json!("2026-11-05T09:01:00Z"),
            &json!("2026-11-05T09:05:00Z"),
            "2026-11-05T09:03:00Z",
            300
        ));
        assert!(!evidence_fresh_at(
            &json!("2026-11-05T09:01:00Z"),
            &json!("2026-11-05T09:02:59Z"),
            "2026-11-05T09:03:00Z",
            300
        ));
        assert!(!evidence_fresh_at(
            &json!("2026-11-05T09:03:01Z"),
            &json!("2026-11-05T09:05:00Z"),
            "2026-11-05T09:03:00Z",
            300
        ));
        assert!(!evidence_fresh_at(
            &json!("2026-11-05T08:52:59Z"),
            &json!("2026-11-05T09:10:00Z"),
            "2026-11-05T09:03:00Z",
            600
        ));
    }

    #[test]
    fn sha256_known_vector_is_correct() {
        let mut hasher = Sha256::new();
        hasher.update(b"abc");
        assert_eq!(
            as_hex(&hasher.finalize()),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }
}
