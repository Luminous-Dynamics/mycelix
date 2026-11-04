#![forbid(unsafe_code)]

//! Effects-disabled evaluator for typed, contestable, privacy-minimised fraud signals.
//! It never scores customers or grants a payment decision to a source signal.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-fraud-signal-corpus-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/sa-payment-fraud-signal-corpus-v1.schema.json");
const EXPECTED_SCHEMA: &str = "mycelix.sa-payment-fraud-signal/v1";
const EXPECTED_CORPUS: &str = "ZA-SA-PAYMENT-FRAUD-SIGNAL-SYNTHETIC-V1";
const AS_OF: &str = "2026-11-04T23:59:59Z";

fn str_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
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
        Ok(validator) => validator,
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

fn parse_utc_seconds(value: &str) -> Option<i64> {
    if value.len() != 20
        || value.as_bytes().get(10) != Some(&b'T')
        || value.as_bytes().get(19) != Some(&b'Z')
    {
        return None;
    }
    let year = value[0..4].parse::<i64>().ok()?;
    let month = value[5..7].parse::<i64>().ok()?;
    let day = value[8..10].parse::<i64>().ok()?;
    let hour = value[11..13].parse::<i64>().ok()?;
    let minute = value[14..16].parse::<i64>().ok()?;
    let second = value[17..19].parse::<i64>().ok()?;
    if !(1..=12).contains(&month)
        || !(0..=23).contains(&hour)
        || !(0..=59).contains(&minute)
        || !(0..=59).contains(&second)
    {
        return None;
    }
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let max_day = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return None,
    };
    if !(1..=max_day).contains(&day) {
        return None;
    }
    let adjusted_year = year - if month <= 2 { 1 } else { 0 };
    let era = adjusted_year.div_euclid(400);
    let year_of_era = adjusted_year - era * 400;
    let adjusted_month = month + if month > 2 { -3 } else { 9 };
    let day_of_year = (153 * adjusted_month + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    let days = era * 146_097 + day_of_era - 719_468;
    Some(days * 86_400 + hour * 3_600 + minute * 60 + second)
}

fn optional_utc_seconds(value: &Value, key: &str) -> Option<Option<i64>> {
    let field = value.get(key)?;
    if field.is_null() {
        Some(None)
    } else {
        Some(str_at(value, key).and_then(parse_utc_seconds))
    }
}

fn is_current_status(status: &str) -> bool {
    matches!(
        status,
        "suspected" | "corroborated" | "confirmed_by_source" | "contested" | "under_review"
    )
}

fn semantic_errors(corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if str_at(corpus, "schema") != Some(EXPECTED_SCHEMA) {
        errors.push("unexpected corpus schema identity".into());
    }
    if str_at(corpus, "corpus_id") != Some(EXPECTED_CORPUS) {
        errors.push("unexpected corpus identity".into());
    }
    if str_at(corpus, "status") != Some("synthetic_reference_only") {
        errors.push("corpus must remain synthetic_reference_only".into());
    }
    if corpus
        .get("external_effects_enabled")
        .and_then(Value::as_bool)
        != Some(false)
    {
        errors.push("external_effects_enabled must remain false".into());
    }
    let Some(as_of) = str_at(corpus, "as_of") else {
        errors.push("missing as_of date".into());
        return errors;
    };
    if as_of != "2026-11-04" {
        errors.push(format!("unexpected frozen as_of date {as_of}"));
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
        .filter_map(|v| str_at(v, "id").map(str::to_owned))
        .collect();
    let profile_map: BTreeMap<String, Value> = profiles
        .iter()
        .filter_map(|v| str_at(v, "profile_id").map(|id| (id.to_owned(), v.clone())))
        .collect();

    for (index, profile) in profiles.iter().enumerate() {
        let jurisdiction = str_at(profile, "jurisdiction").unwrap_or("");
        let notification_rule = str_at(profile, "notification_rule_profile").unwrap_or("");
        let rule_is_compatible = match jurisdiction {
            "ZA" => notification_rule == "ZA_POPIA_S24_DECISION_IMPACT_REASONABLY_PRACTICABLE",
            "MULTI" => notification_rule == "UNQUALIFIED_MULTI_JURISDICTION",
            "EU" => notification_rule == "EU_GDPR_ARTICLE_19_COMPARATOR",
            _ => notification_rule == "UNQUALIFIED_MULTI_JURISDICTION",
        };
        if !rule_is_compatible {
            errors.push(format!("profiles[{index}]: notification rule profile does not match the declared jurisdiction"));
        }
        let refs = array_at(profile, "source_refs")
            .cloned()
            .unwrap_or_default();
        if refs.is_empty() {
            errors.push(format!("profiles[{index}]: no source references"));
        }
        for reference in refs {
            if !reference.as_str().is_some_and(|id| source_ids.contains(id)) {
                errors.push(format!("profiles[{index}]: unresolved source reference"));
            }
        }
        if str_at(profile, "profile_status") == Some("proposal_only")
            && (!array_at(profile, "allowed_source_participants").is_none_or(Vec::is_empty)
                || !array_at(profile, "allowed_audiences").is_none_or(Vec::is_empty))
        {
            errors.push(format!("profiles[{index}]: proposal-only profile cannot authorise operational sources/audiences"));
        }
    }

    for scenario in scenarios.iter() {
        let case_id = str_at(scenario, "scenario_id").unwrap_or("<missing>");
        let signal = &scenario["signal"];
        let profile_id = str_at(signal, "profile_id").unwrap_or("");
        let Some(profile) = profile_map.get(profile_id) else {
            errors.push(format!("{case_id}: unresolved profile {profile_id}"));
            continue;
        };
        let source = str_at(signal, "source_participant").unwrap_or("");
        let source_state = str_at(signal, "source_authority_state").unwrap_or("");
        let signal_jurisdiction = str_at(signal, "jurisdiction").unwrap_or("");
        let profile_jurisdiction = str_at(profile, "jurisdiction").unwrap_or("");
        let disclosures_empty = array_at(scenario, "disclosures").is_none_or(Vec::is_empty);
        let decision_action = str_at(&scenario["receiver_decision"], "action").unwrap_or("");
        if profile_jurisdiction != "MULTI"
            && profile_jurisdiction != signal_jurisdiction
            && (!disclosures_empty
                || decision_action != "none"
                || str_at(scenario, "expected_disposition")
                    != Some("quarantine_cross_jurisdiction"))
        {
            errors.push(format!(
                "{case_id}: cross-jurisdiction use lacks an explicit quarantine disposition"
            ));
        }
        let profile_status = str_at(profile, "profile_status").unwrap_or("");
        let allowed_sources = array_at(profile, "allowed_source_participants");
        let source_allowed =
            allowed_sources.is_some_and(|items| items.iter().any(|v| v.as_str() == Some(source)));
        if profile_status == "proposal_only"
            && source_state == "authorized"
            && is_current_status(str_at(signal, "status").unwrap_or(""))
        {
            errors.push(format!("{case_id}: proposal-only programme profile cannot produce current operational signal"));
        }
        if source_state == "authorized" && !source_allowed {
            errors.push(format!(
                "{case_id}: source participant is not admitted by this profile"
            ));
        }
        if source_state != "authorized" {
            if str_at(signal, "status") != Some("unknown") {
                errors.push(format!(
                    "{case_id}: unknown/revoked source signal must remain unknown"
                ));
            }
            if !array_at(scenario, "disclosures").is_none_or(Vec::is_empty) {
                errors.push(format!(
                    "{case_id}: unknown/revoked source must not disclose a signal"
                ));
            }
            if !matches!(
                str_at(scenario, "expected_disposition"),
                Some(
                    "quarantine_source"
                        | "quarantine_revoked_source"
                        | "quarantine_missing_provenance"
                )
            ) {
                errors.push(format!(
                    "{case_id}: unknown/revoked source lacks quarantine disposition"
                ));
            }
        }

        let purpose = str_at(signal, "purpose").unwrap_or("");
        let allowed_purposes = array_at(profile, "allowed_purposes");
        let purpose_allowed =
            allowed_purposes.is_some_and(|items| items.iter().any(|v| v.as_str() == Some(purpose)));
        if !purpose_allowed && !array_at(scenario, "disclosures").is_none_or(Vec::is_empty) {
            errors.push(format!(
                "{case_id}: out-of-purpose signal cannot be disclosed"
            ));
        }
        if purpose != "payment_fraud_prevention"
            && str_at(scenario, "expected_disposition") != Some("block_secondary_use")
            && str_at(scenario, "expected_disposition") != Some("quarantine_missing_provenance")
        {
            errors.push(format!(
                "{case_id}: signal purpose is not payment fraud prevention"
            ));
        }

        let subject = &signal["subject_reference"];
        let subject_kind = str_at(subject, "kind").unwrap_or("");
        let subject_value = str_at(subject, "value").unwrap_or("");
        if matches!(subject_kind, "public_hash" | "raw_identifier") {
            if !array_at(scenario, "disclosures").is_none_or(Vec::is_empty) {
                errors.push(format!(
                    "{case_id}: public hash/raw identifier cannot be disclosed"
                ));
            }
            if !matches!(
                str_at(scenario, "expected_disposition"),
                Some("reject_unscoped_public_hash" | "block_privacy_leak")
            ) {
                errors.push(format!(
                    "{case_id}: unsafe subject reference lacks privacy rejection disposition"
                ));
            }
        }
        if subject_kind == "public_hash"
            && subject_value.starts_with("sha256:")
            && str_at(scenario, "expected_disposition") != Some("reject_unscoped_public_hash")
        {
            errors.push(format!(
                "{case_id}: bare public SHA-256 is not an acceptable subject pseudonym"
            ));
        }
        if subject_kind == "provider_opaque_token" && !subject_value.starts_with("tok:") {
            errors.push(format!(
                "{case_id}: provider opaque token must use the synthetic token namespace"
            ));
        }

        let status = str_at(signal, "status").unwrap_or("");
        let observed_at = str_at(signal, "observed_at").unwrap_or("");
        let recorded_at = str_at(signal, "recorded_at").unwrap_or("");
        let expiry = str_at(signal, "expires_at").unwrap_or("");
        let observed_seconds = parse_utc_seconds(observed_at);
        let recorded_seconds = parse_utc_seconds(recorded_at);
        let expiry_seconds = parse_utc_seconds(expiry);
        if observed_seconds.is_none() || recorded_seconds.is_none() || expiry_seconds.is_none() {
            errors.push(format!(
                "{case_id}: signal timestamps must be valid UTC RFC3339 seconds"
            ));
        } else if let (Some(observed), Some(recorded), Some(expires)) =
            (observed_seconds, recorded_seconds, expiry_seconds)
        {
            if recorded < observed || expires <= observed {
                errors.push(format!(
                    "{case_id}: recorded/expiry time precedes signal observation"
                ));
            }
            if let Some(max_ttl) = profile
                .get("max_signal_ttl_seconds")
                .and_then(Value::as_u64)
                && expires - observed > max_ttl as i64
            {
                errors.push(format!(
                    "{case_id}: signal lifetime exceeds profile maximum TTL"
                ));
            }
            if expires <= parse_utc_seconds(AS_OF).unwrap_or(i64::MAX) && status != "expired" {
                errors.push(format!("{case_id}: expired signal is still marked current"));
            }
        }
        if matches!(status, "withdrawn" | "corrected" | "expired" | "unknown") {
            if !array_at(scenario, "disclosures").is_none_or(Vec::is_empty) {
                errors.push(format!(
                    "{case_id}: withdrawn/corrected/expired/unknown signal was disclosed as current"
                ));
            }
        }

        let refs = array_at(signal, "evidence_refs")
            .cloned()
            .unwrap_or_default();
        if !refs
            .iter()
            .any(|reference| str_at(reference, "participant_id") == Some(source))
        {
            errors.push(format!(
                "{case_id}: primary source participant is absent from evidence lineage"
            ));
        }
        if source_state == "authorized" {
            for reference in &refs {
                let participant = str_at(reference, "participant_id").unwrap_or("");
                if !allowed_sources.is_some_and(|items| {
                    items.iter().any(|item| item.as_str() == Some(participant))
                }) {
                    errors.push(format!("{case_id}: evidence reference participant is outside the admitted source profile"));
                }
            }
        }
        let independent_roots: BTreeSet<String> = refs
            .iter()
            .filter(|reference| str_at(reference, "independence") == Some("independent"))
            .filter_map(|reference| str_at(reference, "root_event_id").map(str::to_owned))
            .collect();
        let independent_participants: BTreeSet<String> = refs
            .iter()
            .filter(|reference| str_at(reference, "independence") == Some("independent"))
            .filter_map(|reference| str_at(reference, "participant_id").map(str::to_owned))
            .collect();
        if status == "corroborated"
            && (independent_roots.len() < 2 || independent_participants.len() < 2)
        {
            errors.push(format!("{case_id}: corroborated status requires at least two independent roots and participants"));
        }
        if status == "confirmed_by_source" {
            let source_confirmation = refs.iter().any(|reference| {
                str_at(reference, "evidence_class") == Some("source_final_case_disposition")
                    && str_at(reference, "participant_id") == Some(source)
            }) && str_at(signal, "evidence_class")
                == Some("source_final_case_disposition");
            if !source_confirmation || source_state != "authorized" {
                errors.push(format!("{case_id}: confirmed_by_source lacks an authorised source final case disposition"));
            }
        }

        let events = array_at(scenario, "events").cloned().unwrap_or_default();
        let mut event_ids = BTreeSet::new();
        let mut prior_sequence = 0_u64;
        let mut previous_event_version = 0_u64;
        let mut previous_event_digest: Option<String> = None;
        let mut last_event_version = 0_u64;
        let mut last_event_digest: Option<String> = None;
        let mut last_state: Option<String> = None;
        let mut seen_confirmation = false;
        let mut seen_correction = false;
        let mut previous_event_seconds: Option<i64> = None;
        let mut event_digest_by_version: BTreeMap<u64, String> = BTreeMap::new();
        let mut seen_withdrawal = false;
        let mut seen_appeal = false;
        for (event_index, event) in events.iter().enumerate() {
            let path = format!("{case_id}.events[{event_index}]");
            let event_id = str_at(event, "event_id").unwrap_or("<missing>");
            if !event_ids.insert(event_id.to_owned()) {
                errors.push(format!("{path}: duplicate event_id {event_id}"));
            }
            let sequence = event.get("sequence").and_then(Value::as_u64).unwrap_or(0);
            if sequence <= prior_sequence {
                errors.push(format!(
                    "{path}: event sequence must be strictly increasing"
                ));
            }
            prior_sequence = sequence;
            let event_version = event
                .get("signal_version")
                .and_then(Value::as_u64)
                .unwrap_or(0);
            let event_digest = str_at(event, "signal_digest").unwrap_or("");
            let signal_version = signal
                .get("signal_version")
                .and_then(Value::as_u64)
                .unwrap_or(0);
            if event_version == 0
                || event_version < previous_event_version
                || event_version > signal_version
            {
                errors.push(format!("{path}: event signal version is missing, decreases or exceeds the current signal"));
            }
            if previous_event_version > 0 && event_version > previous_event_version + 1 {
                errors.push(format!("{path}: signal version skips a revision"));
            }
            if event_version == previous_event_version
                && previous_event_digest
                    .as_deref()
                    .is_some_and(|digest| digest != event_digest)
            {
                errors.push(format!(
                    "{path}: content digest changed without advancing the signal version"
                ));
            }
            if event_version > previous_event_version
                && previous_event_version > 0
                && !matches!(
                    str_at(event, "event_type"),
                    Some(
                        "signal_corrected"
                            | "signal_withdrawn"
                            | "appeal_resolved_correction"
                            | "source_correction_recorded"
                            | "source_finding_overturned"
                    )
                )
            {
                errors.push(format!(
                    "{path}: signal version advanced without a correction/withdrawal event"
                ));
            }
            if event_version > 0 {
                event_digest_by_version
                    .entry(event_version)
                    .or_insert_with(|| event_digest.to_owned());
            }
            previous_event_version = event_version;
            previous_event_digest = Some(event_digest.to_owned());
            last_event_version = event_version;
            last_event_digest = Some(event_digest.to_owned());
            if event_version == signal_version
                && event_digest != str_at(signal, "evidence_digest").unwrap_or("")
            {
                errors.push(format!(
                    "{path}: event digest does not bind the current signal version"
                ));
            }
            if str_at(event, "authority_state") == Some("revoked")
                && str_at(event, "event_type") != Some("signal_withdrawn")
            {
                errors.push(format!(
                    "{path}: revoked source cannot emit an accepted status event"
                ));
            }
            let observed = str_at(event, "observed_at").unwrap_or("");
            let event_type = str_at(event, "event_type").unwrap_or("");
            let authority_state = str_at(event, "authority_state").unwrap_or("");
            let blocked_clock_skew = event_type == "clock_skew_quarantined"
                && authority_state == "unknown"
                && event.get("to_status").is_none_or(Value::is_null);
            let event_seconds = parse_utc_seconds(observed);
            if event_seconds.is_none() {
                errors.push(format!(
                    "{path}: event timestamp is not valid UTC RFC3339 seconds"
                ));
            } else if let Some(current) = event_seconds {
                if previous_event_seconds.is_some_and(|prior| current < prior) {
                    errors.push(format!("{path}: event time moves backwards"));
                }
                previous_event_seconds = Some(current);
                if current > parse_utc_seconds(AS_OF).unwrap_or(i64::MAX) && !blocked_clock_skew {
                    errors.push(format!("{path}: future event is not quarantined"));
                }
            }
            if let Some(to_status) = event.get("to_status").and_then(Value::as_str) {
                last_state = Some(to_status.to_owned());
                if to_status == "confirmed_by_source" {
                    seen_confirmation = true;
                }
                if to_status == "corrected" {
                    seen_correction = true;
                }
                if to_status == "withdrawn" {
                    seen_withdrawal = true;
                }
            }
            if str_at(event, "event_type").is_some_and(|k| k.contains("appeal")) {
                seen_appeal = true;
            }
            if str_at(event, "event_type") == Some("evidence_digest_mutation_blocked")
                && event
                    .get("attempted_digest")
                    .and_then(Value::as_str)
                    .is_none()
            {
                errors.push(format!(
                    "{path}: blocked digest mutation lacks attempted digest"
                ));
            }
        }
        if last_state.as_deref() != Some(status) {
            errors.push(format!(
                "{case_id}: final signal status does not match append-only event history"
            ));
        }
        let current_signal_version = signal
            .get("signal_version")
            .and_then(Value::as_u64)
            .unwrap_or(0);
        if last_event_version != current_signal_version
            || last_event_digest.as_deref() != str_at(signal, "evidence_digest")
        {
            errors.push(format!(
                "{case_id}: final history event does not bind the current version and digest"
            ));
        }
        let historical_disclosures = array_at(scenario, "historical_disclosures")
            .cloned()
            .unwrap_or_default();
        let propagation_obligations = array_at(scenario, "correction_propagations")
            .cloned()
            .unwrap_or_default();
        let current_version = signal
            .get("signal_version")
            .and_then(Value::as_u64)
            .unwrap_or(0);
        let current_digest = str_at(signal, "evidence_digest").unwrap_or("");
        let rule_profile = str_at(profile, "notification_rule_profile").unwrap_or("");
        let inventory = &scenario["historical_disclosure_inventory"];
        let inventory_status = str_at(inventory, "status").unwrap_or("");
        let inventory_count = inventory
            .get("recipient_count")
            .and_then(Value::as_u64)
            .unwrap_or(u64::MAX);
        let inventory_evidence = str_at(inventory, "evidence_reference").unwrap_or("");
        let inventory_verified_at = optional_utc_seconds(inventory, "verified_at");
        let current_signal_changed = current_version > 1
            && matches!(status, "corrected" | "withdrawn" | "expired");
        let inventory_complete = inventory_status == "complete";
        if inventory_status.is_empty() || inventory_evidence.is_empty() {
            errors.push(format!("{case_id}: historical disclosure inventory lacks status or evidence reference"));
        }
        if inventory_verified_at.is_none() {
            errors.push(format!("{case_id}: historical disclosure inventory verified_at is malformed"));
        }
        if inventory_complete {
            if inventory_count != historical_disclosures.len() as u64
                || inventory_verified_at.flatten().is_none()
            {
                errors.push(format!("{case_id}: complete recipient inventory count/timestamp does not reconcile"));
            }
        } else if inventory_status == "not_applicable" {
            if current_signal_changed
                || !historical_disclosures.is_empty()
                || inventory_count != 0
                || inventory_verified_at.flatten().is_some()
            {
                errors.push(format!("{case_id}: not_applicable inventory conflicts with a superseded signal or historical disclosure"));
            }
        } else if !matches!(inventory_status, "incomplete" | "unknown") {
            errors.push(format!("{case_id}: unknown historical disclosure inventory state"));
        }

        let mut historical_keys = BTreeSet::new();
        for (history_index, disclosure) in historical_disclosures.iter().enumerate() {
            let recipient = str_at(disclosure, "recipient_participant").unwrap_or("");
            let prior_version = disclosure
                .get("signal_version")
                .and_then(Value::as_u64)
                .unwrap_or(0);
            let prior_digest = str_at(disclosure, "signal_digest").unwrap_or("");
            let key = format!("{recipient}|{prior_version}|{prior_digest}");
            if !historical_keys.insert(key.clone()) {
                errors.push(format!("{case_id}: duplicate historical disclosure {key}"));
            }
            if prior_version == 0 || prior_version >= current_version {
                errors.push(format!("{case_id}: historical disclosure must reference a superseded signal version"));
            }
            if event_digest_by_version.get(&prior_version).map(String::as_str)
                != Some(prior_digest)
            {
                errors.push(format!("{case_id}: historical disclosure digest does not match the source event history"));
            }
            if str_at(disclosure, "purpose") != Some("payment_fraud_prevention")
                || str_at(disclosure, "audience") != Some("participating_psp_fraud_team")
                || recipient.is_empty()
            {
                errors.push(format!("{case_id}: historical disclosure has invalid purpose, audience or recipient"));
            }
            if history_index >= historical_disclosures.len() {
                errors.push(format!("{case_id}: unreachable historical disclosure index"));
            }
        }

        let mut obligation_keys = BTreeSet::new();
        let mut unresolved_propagation = false;
        let mut all_propagation_invalidated = !propagation_obligations.is_empty();
        let mut all_propagation_not_required = !propagation_obligations.is_empty();
        let mut any_exception_recorded = false;
        let mut any_ack_without_invalidation = false;
        for (obligation_index, obligation) in propagation_obligations.iter().enumerate() {
            let opath = format!("{case_id}.correction_propagations[{obligation_index}]");
            let recipient = str_at(obligation, "recipient_participant").unwrap_or("");
            let audience = str_at(obligation, "recipient_audience").unwrap_or("");
            let prior_version = obligation
                .get("prior_signal_version")
                .and_then(Value::as_u64)
                .unwrap_or(0);
            let prior_digest = str_at(obligation, "prior_signal_digest").unwrap_or("");
            let new_version = obligation
                .get("new_signal_version")
                .and_then(Value::as_u64)
                .unwrap_or(0);
            let new_digest = str_at(obligation, "new_signal_digest").unwrap_or("");
            let key = format!("{recipient}|{prior_version}|{prior_digest}");
            if !obligation_keys.insert(key.clone()) {
                errors.push(format!("{opath}: duplicate propagation obligation {key}"));
            }
            if !historical_keys.contains(&key) {
                errors.push(format!("{opath}: propagation obligation has no matching historical disclosure"));
            }
            if new_version != current_version || new_digest != current_digest {
                errors.push(format!("{opath}: propagation notice is not bound to the current signal version and digest"));
            }
            if event_digest_by_version.get(&prior_version).map(String::as_str)
                != Some(prior_digest)
            {
                errors.push(format!("{opath}: prior digest does not match signal history"));
            }
            if !historical_disclosures.iter().any(|disclosure| {
                str_at(disclosure, "recipient_participant") == Some(recipient)
                    && str_at(disclosure, "audience") == Some(audience)
                    && disclosure.get("signal_version").and_then(Value::as_u64) == Some(prior_version)
                    && str_at(disclosure, "signal_digest") == Some(prior_digest)
            }) {
                errors.push(format!("{opath}: recipient/audience does not match the historical disclosure"));
            }

            let decision_impact = str_at(obligation, "decision_impact").unwrap_or("");
            let practicability = str_at(obligation, "practicability").unwrap_or("");
            let prop_status = str_at(obligation, "status").unwrap_or("");
            let reason = str_at(obligation, "reason").unwrap_or("");
            let sent = optional_utc_seconds(obligation, "notification_sent_at");
            let acknowledged = optional_utc_seconds(obligation, "acknowledged_at");
            let invalidated = optional_utc_seconds(obligation, "cache_invalidated_at");
            if sent.is_none() || acknowledged.is_none() || invalidated.is_none() {
                errors.push(format!("{opath}: propagation timestamps are malformed"));
            }
            let sent = sent.flatten();
            let acknowledged = acknowledged.flatten();
            let invalidated = invalidated.flatten();
            let notice_receipt = str_at(obligation, "notification_receipt_id");
            let invalidation_receipt = str_at(obligation, "invalidation_receipt_id");
            match prop_status {
                "pending" => {
                    unresolved_propagation = true;
                    if sent.is_some() || acknowledged.is_some() || invalidated.is_some()
                        || notice_receipt.is_some() || invalidation_receipt.is_some()
                    {
                        errors.push(format!("{opath}: pending obligation cannot carry successful delivery/acknowledgement/invalidation evidence"));
                    }
                }
                "delivered" => {
                    unresolved_propagation = true;
                    if sent.is_none() || notice_receipt.is_none()
                        || acknowledged.is_some() || invalidated.is_some()
                        || invalidation_receipt.is_some()
                    {
                        errors.push(format!("{opath}: delivered requires only a notification receipt and send timestamp; it is not invalidation"));
                    }
                }
                "acknowledged" => {
                    unresolved_propagation = true;
                    any_ack_without_invalidation = true;
                    if sent.is_none() || acknowledged.is_none() || notice_receipt.is_none()
                        || invalidated.is_some() || invalidation_receipt.is_some()
                    {
                        errors.push(format!("{opath}: acknowledgement requires notification and acknowledgement evidence but no cache-invalidation claim"));
                    }
                }
                "invalidated" => {
                    all_propagation_not_required = false;
                    if sent.is_none() || acknowledged.is_none() || invalidated.is_none()
                        || notice_receipt.is_none() || invalidation_receipt.is_none()
                    {
                        errors.push(format!("{opath}: invalidation requires send, acknowledgement, cache-invalidation timestamps and both receipts"));
                    }
                    if let (Some(sent_at), Some(ack_at), Some(invalid_at)) =
                        (sent, acknowledged, invalidated)
                        && (sent_at > ack_at || ack_at > invalid_at)
                    {
                        errors.push(format!("{opath}: propagation timestamps are not monotonic"));
                    }
                }
                "not_required_with_basis" => {
                    all_propagation_invalidated = false;
                    if decision_impact != "no" || reason.is_empty()
                        || sent.is_some() || acknowledged.is_some() || invalidated.is_some()
                    {
                        errors.push(format!("{opath}: not-required disposition requires an explicit no-decision-impact assessment and reason"));
                    }
                    if rule_profile == "UNQUALIFIED_MULTI_JURISDICTION" {
                        errors.push(format!("{opath}: unqualified cross-border rule cannot assert not-required legal outcome"));
                    }
                }
                "exception_recorded" => {
                    all_propagation_invalidated = false;
                    all_propagation_not_required = false;
                    any_exception_recorded = true;
                    unresolved_propagation = true;
                    if decision_impact != "yes" || practicability != "not_practicable"
                        || reason.is_empty()
                        || sent.is_some() || acknowledged.is_some() || invalidated.is_some()
                    {
                        errors.push(format!("{opath}: exception requires decision impact, not-practicable assessment and a documented reason"));
                    }
                    if rule_profile == "UNQUALIFIED_MULTI_JURISDICTION" {
                        errors.push(format!("{opath}: unqualified cross-border rule cannot assert a legal notification exception"));
                    }
                }
                "failed" => {
                    all_propagation_invalidated = false;
                    all_propagation_not_required = false;
                    unresolved_propagation = true;
                    if reason.is_empty() {
                        errors.push(format!("{opath}: failed propagation requires a recorded failure reason"));
                    }
                }
                _ => errors.push(format!("{opath}: unknown propagation disposition")),
            }

            if decision_impact == "unknown"
                && prop_status == "not_required_with_basis"
            {
                errors.push(format!("{opath}: unknown decision impact cannot be treated as not required"));
            }
            if practicability == "unknown" && prop_status == "exception_recorded" {
                errors.push(format!("{opath}: unknown practicability cannot be recorded as a concluded exception"));
            }
            if rule_profile == "UNQUALIFIED_MULTI_JURISDICTION"
                && prop_status != "pending"
                && prop_status != "delivered"
                && prop_status != "acknowledged"
                && prop_status != "invalidated"
                && prop_status != "failed"
            {
                errors.push(format!("{opath}: unqualified cross-border profile cannot assert jurisdiction-specific legal disposition"));
            }
        }

        for key in &historical_keys {
            if !obligation_keys.contains(key) {
                errors.push(format!("{case_id}: prior recipient has no correction propagation obligation: {key}"));
            }
        }
        if historical_keys.is_empty() && !propagation_obligations.is_empty() {
            errors.push(format!("{case_id}: propagation obligations exist without a historical recipient disclosure"));
        }
        if current_signal_changed && !inventory_complete {
            unresolved_propagation = true;
            if str_at(scenario, "expected_disposition") != Some("propagation_inventory_incomplete") {
                errors.push(format!("{case_id}: changed signal with incomplete historical recipient inventory must remain quarantined"));
            }
            if !array_at(scenario, "disclosures").is_none_or(Vec::is_empty)
                || !matches!(decision_action, "none" | "manual_review")
            {
                errors.push(format!("{case_id}: incomplete historical inventory cannot support current redistribution or adverse receiver action"));
            }
        } else if str_at(scenario, "expected_disposition") == Some("propagation_inventory_incomplete") {
            errors.push(format!("{case_id}: propagation inventory is marked incomplete despite a complete or not-applicable inventory state"));
        }
        if !propagation_obligations.is_empty() {
            let expected = str_at(scenario, "expected_disposition").unwrap_or("");
            if unresolved_propagation
                && !matches!(expected, "propagation_pending_quarantine" | "propagation_acknowledged_not_invalidated" | "propagation_exception_recorded" | "quarantine_cross_jurisdiction_propagation")
            {
                errors.push(format!("{case_id}: unresolved propagation is not represented in the expected disposition"));
            }
            if !unresolved_propagation && all_propagation_invalidated
                && expected != "propagation_complete"
            {
                errors.push(format!("{case_id}: fully invalidated recipients require propagation_complete disposition"));
            }
            if !unresolved_propagation && all_propagation_not_required
                && expected != "propagation_not_required_reasoned"
            {
                errors.push(format!("{case_id}: no-impact obligations require propagation_not_required_reasoned disposition"));
            }
            if any_exception_recorded && expected != "propagation_exception_recorded" {
                errors.push(format!("{case_id}: documented exception requires propagation_exception_recorded disposition"));
            }
            if any_ack_without_invalidation && expected != "propagation_acknowledged_not_invalidated" {
                errors.push(format!("{case_id}: acknowledgement without invalidation must remain explicitly unresolved"));
            }
            if unresolved_propagation
                && decision_action != "none"
                && decision_action != "manual_review"
            {
                errors.push(format!("{case_id}: receiver cannot take a new adverse action while recipient invalidation is unresolved"));
            }
            if unresolved_propagation && !array_at(scenario, "disclosures").is_none_or(Vec::is_empty) {
                errors.push(format!("{case_id}: unresolved propagation cannot be redistributed as a current signal"));
            }
        }
        if !historical_keys.is_empty()
            && matches!(status, "corrected" | "withdrawn" | "expired")
            && propagation_obligations.is_empty()
        {
            errors.push(format!("{case_id}: superseded signal with prior recipients has no propagation ledger"));
        }

        if status == "confirmed_by_source" && !seen_confirmation {
            errors.push(format!(
                "{case_id}: source-confirmed status lacks a recorded confirmation transition"
            ));
        }
        if matches!(status, "corrected" | "withdrawn")
            && signal
                .get("signal_version")
                .and_then(Value::as_u64)
                .unwrap_or(0)
                < 2
        {
            errors.push(format!(
                "{case_id}: correction/withdrawal must advance the signal version"
            ));
        }
        if status == "corrected" && !seen_correction {
            errors.push(format!(
                "{case_id}: corrected status lacks a correction transition"
            ));
        }
        if status == "withdrawn" && !seen_withdrawal {
            errors.push(format!(
                "{case_id}: withdrawn status lacks a withdrawal transition"
            ));
        }
        if signal
            .get("appeal_state")
            .and_then(Value::as_str)
            .is_some_and(|v| v != "not_submitted")
            && !seen_appeal
        {
            errors.push(format!(
                "{case_id}: appeal state lacks append-only appeal evidence"
            ));
        }

        let profile_audiences = array_at(profile, "allowed_audiences");
        for (disclosure_index, disclosure) in array_at(scenario, "disclosures")
            .cloned()
            .unwrap_or_default()
            .iter()
            .enumerate()
        {
            let dpath = format!("{case_id}.disclosures[{disclosure_index}]");
            let audience = str_at(disclosure, "audience").unwrap_or("");
            let recipient = str_at(disclosure, "recipient_participant").unwrap_or("");
            let disclosure_purpose = str_at(disclosure, "purpose").unwrap_or("");
            let status_at_disclosure = str_at(disclosure, "status_at_disclosure").unwrap_or("");
            let disclosed_version = disclosure.get("signal_version").and_then(Value::as_u64).unwrap_or(0);
            let disclosed_digest = str_at(disclosure, "signal_digest").unwrap_or("");
            let audience_allowed = profile_audiences
                .is_some_and(|items| items.iter().any(|v| v.as_str() == Some(audience)));
            if !audience_allowed || audience != str_at(signal, "audience").unwrap_or("") {
                errors.push(format!("{dpath}: wrong audience"));
            }
            if recipient == source {
                errors.push(format!(
                    "{dpath}: source cannot be treated as an external recipient"
                ));
            }
            if disclosure_purpose != purpose || disclosure_purpose != "payment_fraud_prevention" {
                errors.push(format!("{dpath}: wrong disclosure purpose"));
            }
            if status_at_disclosure != status || !is_current_status(status) {
                errors.push(format!(
                    "{dpath}: disclosure is not bound to current signal status"
                ));
            }
            if disclosed_version
                != signal.get("signal_version").and_then(Value::as_u64).unwrap_or(0)
                || disclosed_digest != str_at(signal, "evidence_digest").unwrap_or("")
            {
                errors.push(format!("{dpath}: disclosure is not bound to the exact current signal version and digest"));
            }
            if disclosure.get("existence_oracle").and_then(Value::as_bool) != Some(false) {
                errors.push(format!(
                    "{dpath}: query must not reveal arbitrary identifier existence"
                ));
            }
            if disclosure
                .get("raw_subject_included")
                .and_then(Value::as_bool)
                != Some(false)
            {
                errors.push(format!(
                    "{dpath}: raw subject identifier cannot be disclosed"
                ));
            }
            let fields = array_at(disclosure, "disclosed_fields")
                .cloned()
                .unwrap_or_default();
            for field in fields {
                if matches!(
                    field.as_str(),
                    Some(
                        "raw_account_number"
                            | "phone_number"
                            | "national_id"
                            | "full_transaction_history"
                            | "credential"
                    )
                ) {
                    errors.push(format!("{dpath}: prohibited disclosure field"));
                }
            }
        }

        let decision = &scenario["receiver_decision"];
        let action = str_at(decision, "action").unwrap_or("");
        let basis = str_at(decision, "basis").unwrap_or("");
        let automatic = decision.get("automatic").and_then(Value::as_bool) == Some(true);
        let independent_review =
            decision.get("independent_review").and_then(Value::as_bool) == Some(true);
        let essential = decision
            .get("essential_access_affected")
            .and_then(Value::as_bool)
            == Some(true);
        if action == "deny_access"
            && (automatic || basis.contains("single_signal") || !independent_review || essential)
        {
            errors.push(format!("{case_id}: adverse access denial cannot be automatic, single-signal or affect essential access"));
        }
        if matches!(status, "contested" | "under_review") && action == "deny_access" {
            errors.push(format!(
                "{case_id}: contested/under-review signal cannot drive access denial"
            ));
        }
        if profile_status == "proposal_only" && action != "none" {
            errors.push(format!(
                "{case_id}: proposal-only framework cannot drive an operational receiver decision"
            ));
        }
        if status == "under_review"
            && signal.get("appeal_state").and_then(Value::as_str) == Some("under_review")
            && action == "deny_access"
        {
            errors.push(format!(
                "{case_id}: unresolved appeal cannot be treated as a final adverse fact"
            ));
        }
    }
    errors
}

fn evaluate(corpus_text: &str, schema_text: &str) -> Result<(Vec<String>, Value, Value), String> {
    let corpus: Value =
        serde_json::from_str(corpus_text).map_err(|error| format!("corpus JSON: {error}"))?;
    let schema: Value =
        serde_json::from_str(schema_text).map_err(|error| format!("schema JSON: {error}"))?;
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
        return Err(format!("baseline fixture must pass: {baseline:?}"));
    }
    let mut count = 0;
    mutate_and_reject("external effects enabled", corpus, schema, |v| {
        v["external_effects_enabled"] = json!(true);
    })?;
    count += 1;
    mutate_and_reject(
        "suspected promoted to confirmed without source evidence",
        corpus,
        schema,
        |v| {
            v["scenarios"][1]["signal"]["status"] = json!("confirmed_by_source");
        },
    )?;
    count += 1;
    mutate_and_reject(
        "duplicate source copy promoted to corroborated",
        corpus,
        schema,
        |v| {
            v["scenarios"][2]["signal"]["status"] = json!("corroborated");
        },
    )?;
    count += 1;
    mutate_and_reject("unauthorised participant source", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["source_participant"] = json!("PSP_X");
    })?;
    count += 1;
    mutate_and_reject("credit scoring secondary use", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["purpose"] = json!("credit_scoring");
    })?;
    count += 1;
    mutate_and_reject("public low-entropy hash", corpus, schema, |v| {
        v["scenarios"][0]["signal"]["subject_reference"]["kind"] = json!("public_hash");
    })?;
    count += 1;
    mutate_and_reject("raw account number disclosure", corpus, schema, |v| {
        v["scenarios"][0]["disclosures"][0]["disclosed_fields"]
            .as_array_mut()
            .unwrap()
            .push(json!("raw_account_number"));
    })?;
    count += 1;
    mutate_and_reject("identifier existence oracle", corpus, schema, |v| {
        v["scenarios"][0]["disclosures"][0]["existence_oracle"] = json!(true);
    })?;
    count += 1;
    mutate_and_reject(
        "withdrawn signal disclosed as current",
        corpus,
        schema,
        |v| {
            v["scenarios"][10]["disclosures"] = v["scenarios"][0]["disclosures"].clone();
        },
    )?;
    count += 1;
    mutate_and_reject("single signal automatic denial", corpus, schema, |v| {
        v["scenarios"][0]["receiver_decision"]["action"] = json!("deny_access");
        v["scenarios"][0]["receiver_decision"]["automatic"] = json!(true);
        v["scenarios"][0]["receiver_decision"]["basis"] = json!("single_signal");
    })?;
    count += 1;
    mutate_and_reject("stale signal remains current", corpus, schema, |v| {
        v["scenarios"][9]["signal"]["status"] = json!("suspected");
    })?;
    count += 1;
    mutate_and_reject("wrong disclosure audience", corpus, schema, |v| {
        v["scenarios"][0]["disclosures"][0]["audience"] = json!("credit_bureau");
    })?;
    count += 1;
    mutate_and_reject(
        "signal content changes under the same version",
        corpus,
        schema,
        |v| {
            v["scenarios"][0]["signal"]["evidence_digest"] =
                json!("sha256:2222222222222222222222222222222222222222222222222222222222222222");
        },
    )?;
    count += 1;
    mutate_and_reject(
        "signal TTL exceeds participant profile",
        corpus,
        schema,
        |v| {
            v["scenarios"][0]["signal"]["expires_at"] = json!("2026-11-12T09:00:00Z");
        },
    )?;
    count += 1;
    mutate_and_reject("history version skips a revision", corpus, schema, |v| {
        v["scenarios"][11]["events"][1]["signal_version"] = json!(3);
    })?;
    count += 1;
    mutate_and_reject("missing recipient propagation obligation", corpus, schema, |v| {
        v["scenarios"][33]["correction_propagations"] = json!([]);
    })?;
    count += 1;
    mutate_and_reject("delivery receipt promoted to cache invalidation", corpus, schema, |v| {
        v["scenarios"][34]["correction_propagations"][0]["status"] = json!("invalidated");
    })?;
    count += 1;
    mutate_and_reject("wrong correction digest acknowledged", corpus, schema, |v| {
        v["scenarios"][33]["correction_propagations"][0]["new_signal_digest"] = json!("sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    })?;
    count += 1;
    mutate_and_reject("no-impact claim without reason", corpus, schema, |v| {
        v["scenarios"][35]["correction_propagations"][0]["reason"] = Value::Null;
    })?;
    count += 1;
    mutate_and_reject("acknowledgement treated as complete invalidation", corpus, schema, |v| {
        v["scenarios"][37]["expected_disposition"] = json!("propagation_complete");
    })?;
    count += 1;
    mutate_and_reject("GDPR rule profile attached to South African participant", corpus, schema, |v| {
        v["profiles"][0]["notification_rule_profile"] = json!("EU_GDPR_ARTICLE_19_COMPARATOR");
    })?;
    count += 1;
    mutate_and_reject("unqualified cross-border rule claims a legal exception", corpus, schema, |v| {
        v["scenarios"][38]["correction_propagations"][0]["status"] = json!("not_required_with_basis");
        v["scenarios"][38]["correction_propagations"][0]["decision_impact"] = json!("no");
        v["scenarios"][38]["correction_propagations"][0]["reason"] = json!("unknown jurisdiction");
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
    let report = json!({
        "evaluator":"mycelix.sa-payment-fraud-signal-evaluator.v1",
        "status":if errors.is_empty(){"PASS"}else{"FAIL"},
        "external_effects_enabled":false,
        "claim_ceiling":"synthetic signal provenance, scope, privacy and lifecycle semantics only",
        "source_count":array_at(&corpus,"source_register").map_or(0,Vec::len),
        "profile_count":array_at(&corpus,"profiles").map_or(0,Vec::len),
        "scenario_count":array_at(&corpus,"scenarios").map_or(0,Vec::len),
        "negative_controls":mutation_count,
        "errors":errors
    });
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
    fn checked_in_schema_and_corpus_validate() {
        let (errors, corpus, _) = evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("evaluation");
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(array_at(&corpus, "source_register").unwrap().len(), 4);
        assert_eq!(array_at(&corpus, "profiles").unwrap().len(), 3);
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 33);
    }

    #[test]
    fn negative_mutation_campaign_rejects_privacy_and_status_laundering() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self-tests"), 22);
    }
}
