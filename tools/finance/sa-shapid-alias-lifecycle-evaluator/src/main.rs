#![forbid(unsafe_code)]

//! Effects-disabled reference evaluator for South African ShapID alias lifecycle.
//! A phone number, SIM control, bank alias and bank-account ownership are distinct facts.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::BTreeSet;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/sa-shapid-alias-lifecycle-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/sa-shapid-alias-lifecycle-v1.schema.json");

fn str_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn u64_at(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn array_at<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn has_evidence(event: &Value, expected: &str) -> bool {
    array_at(event, "evidence_types")
        .is_some_and(|items| items.iter().any(|item| item.as_str() == Some(expected)))
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

fn date_time_after(value: &str, as_of: &str) -> bool {
    valid_date(value.get(..10).unwrap_or(""))
        && value.len() >= 20
        && value.as_bytes().get(10) == Some(&b'T')
        && value > format!("{as_of}T23:59:59Z").as_str()
}

fn schema_errors(schema_text: &str, corpus_text: &str) -> Result<(Value, Value), Vec<String>> {
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
    let errors = validator
        .iter_errors(&corpus)
        .map(|error| format!("{}: {error}", error.instance_path()))
        .collect::<Vec<_>>();
    if errors.is_empty() {
        Ok((schema, corpus))
    } else {
        Err(errors)
    }
}

fn unique_ids(items: &[Value], key: &str, label: &str, errors: &mut Vec<String>) {
    let mut seen = BTreeSet::new();
    for (index, item) in items.iter().enumerate() {
        match str_at(item, key) {
            Some(id) if seen.insert(id.to_owned()) => {}
            Some(id) => errors.push(format!("{label}[{index}].{key}: duplicate {id}")),
            None => errors.push(format!("{label}[{index}].{key}: missing string")),
        }
    }
}

fn compute_state(case: &Value, profile: &Value, as_of: &str) -> (&'static str, bool, bool) {
    let alias = &case["alias"];
    let generation = u64_at(alias, "generation").unwrap_or(0);
    let beneficiary = str_at(alias, "beneficiary_commitment").unwrap_or("");
    let mut registered = false;
    let mut owner_verified = false;
    let mut account_binding_verified = false;
    let mut provider_confirmed = false;
    let mut resolver_fresh = false;
    let mut resolver_epoch = 0_u64;
    let mut resolver_valid_until = String::new();
    let mut current_epoch = 0_u64;
    let mut revoked = false;
    let mut change_pending = false;
    let mut reassignment = false;
    let mut ownership_uncertain = false;
    let mut default_conflict = false;
    let mut conflict = false;
    let mut fingerprint_conflict = false;
    let mut policy_unqualified = false;
    let mut stale = false;
    let mut rate_limited = false;
    let mut payer_mismatch = false;
    let mut payer_intent_presented = false;
    let mut historical_payment = false;
    let mut rebind_required = false;

    let events = array_at(case, "events").map(Vec::as_slice).unwrap_or(&[]);
    for event in events {
        let event_type = str_at(event, "event_type").unwrap_or("");
        let authority = str_at(event, "authority_state").unwrap_or("unknown");
        let event_generation = u64_at(event, "generation");
        let event_beneficiary = str_at(event, "beneficiary_commitment").unwrap_or("");
        let identity_matches = event_generation == Some(generation)
            && (event_beneficiary.is_empty() || event_beneficiary == beneficiary);
        let valid_authority = authority == "valid";

        match event_type {
            "alias_rebound" => {
                registered = false;
                owner_verified = false;
                account_binding_verified = false;
                provider_confirmed = false;
                resolver_fresh = false;
                resolver_epoch = 0;
                resolver_valid_until.clear();
                rebind_required = true;
            }
            "alias_registered" => {
                registered = valid_authority
                    && identity_matches
                    && has_evidence(event, "alias_registration_receipt");
                if registered {
                    rebind_required = false;
                }
            }
            "alias_owner_verified" => {
                owner_verified = valid_authority
                    && identity_matches
                    && has_evidence(event, "provider_identity_match")
                    && has_evidence(event, "alias_control_verified");
            }
            "bank_account_binding_verified" => {
                account_binding_verified = valid_authority
                    && identity_matches
                    && has_evidence(event, "bank_account_binding_receipt");
            }
            "provider_registration_confirmed" => {
                provider_confirmed = valid_authority
                    && identity_matches
                    && has_evidence(event, "provider_registration_receipt");
            }
            "resolver_result" => {
                let exact_replay = has_evidence(event, "exact_same_generation_fingerprint_replay")
                    && resolver_fresh
                    && event_generation == Some(generation)
                    && event_beneficiary == beneficiary
                    && str_at(event, "freshness") == Some("fresh")
                    && str_at(event, "valid_until") == Some(resolver_valid_until.as_str())
                    && u64_at(event, "directory_epoch") == Some(resolver_epoch);
                if exact_replay {
                    // Exact same-generation/fingerprint replay is idempotent: it does not
                    // refresh the original validity window or invalidate the prior result.
                } else if event_generation.is_some_and(|value| value != generation)
                    || (!event_beneficiary.is_empty() && event_beneficiary != beneficiary)
                {
                    conflict = true;
                } else if !valid_authority
                    || str_at(event, "freshness") != Some("fresh")
                    || !str_at(event, "valid_until")
                        .is_some_and(|value| date_time_after(value, as_of))
                    || !has_evidence(event, "authenticated_directory_response")
                {
                    stale = true;
                    resolver_fresh = false;
                } else {
                    resolver_fresh = true;
                    resolver_epoch = u64_at(event, "directory_epoch").unwrap_or(0);
                    resolver_valid_until = str_at(event, "valid_until").unwrap_or("").to_owned();
                }
            }
            "directory_epoch_advanced" => {
                current_epoch = current_epoch.max(u64_at(event, "directory_epoch").unwrap_or(0));
            }
            "phone_number_changed" => change_pending = true,
            "number_release_reported" | "number_reassigned" | "sim_swap_signal" => {
                reassignment = true;
            }
            "number_ported" => ownership_uncertain = true,
            "alias_revoked" | "provider_deregistration" | "account_closed" => revoked = true,
            "alias_suspended" => rebind_required = true,
            "alias_reactivated" => {
                if event_generation.is_none_or(|value| value <= generation) {
                    rebind_required = true;
                }
            }
            "default_alias_conflict" => default_conflict = true,
            "cross_participant_binding_conflict" => conflict = true,
            "alias_fingerprint_changed" => fingerprint_conflict = true,
            "policy_claim_used" => {
                policy_unqualified |= has_evidence(event, "draft_fifth_amendment_cited_as_final")
                    || has_evidence(event, "operator_cooldown_not_in_source");
            }
            "payer_display_mismatch" => payer_mismatch = true,
            "payer_intent_presented" | "payer_confirmation_attempted" => {
                payer_intent_presented = true;
            }
            "payment_authorisation_recorded" => historical_payment = true,
            "alias_enumeration_attempt" => {
                rate_limited = has_evidence(event, "rate_limit_exceeded")
                    && has_evidence(event, "no_account_disclosure");
            }
            _ => {}
        }
    }

    if resolver_epoch < current_epoch {
        stale = true;
    }

    let invalidated = revoked || reassignment || change_pending;
    if historical_payment && invalidated {
        return ("historical_binding_only", false, false);
    }
    if policy_unqualified {
        return ("policy_claim_unqualified", false, false);
    }
    if rate_limited {
        return ("rate_limited", false, false);
    }
    if default_conflict {
        return ("default_conflict", false, false);
    }
    if fingerprint_conflict {
        return ("fingerprint_conflict", false, false);
    }
    if conflict {
        return ("conflict", false, false);
    }
    if reassignment {
        return ("reassignment_quarantine", false, false);
    }
    if change_pending && payer_intent_presented {
        return ("reauthorisation_required", false, true);
    }
    if change_pending {
        return ("change_pending_quarantine", false, false);
    }
    if payer_mismatch {
        return ("reauthorisation_required", false, true);
    }
    if revoked {
        return ("revoked", false, false);
    }
    if rebind_required {
        return ("rebind_required", false, false);
    }
    if stale {
        return ("stale", false, false);
    }
    if ownership_uncertain
        || !registered
        || !owner_verified
        || !account_binding_verified
        || !provider_confirmed
        || !resolver_fresh
    {
        return ("ownership_unverified", false, false);
    }

    let _profile_id = str_at(profile, "profile_id").unwrap_or("");
    ("active_resolved", true, false)
}

fn semantic_errors(corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if str_at(corpus, "schema") != Some("mycelix.sa-payment-alias-lifecycle/v1") {
        errors.push("unexpected corpus schema identity".into());
    }
    if str_at(corpus, "status") != Some("synthetic_reference_only") {
        errors.push("corpus must remain synthetic_reference_only".into());
    }
    if corpus
        .get("external_effects_enabled")
        .and_then(Value::as_bool)
        != Some(false)
        || corpus.get("dispatch_enabled").and_then(Value::as_bool) != Some(false)
    {
        errors.push("external effects and dispatch must remain disabled".into());
    }
    let as_of = str_at(corpus, "as_of").unwrap_or("");
    if !valid_date(as_of) {
        errors.push("as_of must be a valid frozen date".into());
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
    let profile_map: std::collections::BTreeMap<String, Value> = profiles
        .iter()
        .filter_map(|profile| {
            str_at(profile, "profile_id").map(|id| (id.to_owned(), profile.clone()))
        })
        .collect();

    for (index, profile) in profiles.iter().enumerate() {
        let refs = array_at(profile, "source_refs")
            .cloned()
            .unwrap_or_default();
        if refs.is_empty() {
            errors.push(format!("profiles[{index}].source_refs must not be empty"));
        }
        for reference in refs {
            if !reference.as_str().is_some_and(|id| source_ids.contains(id)) {
                errors.push(format!(
                    "profiles[{index}].source_refs contains unresolved source"
                ));
            }
        }
        if str_at(profile, "participant") == Some("ICASA")
            && str_at(profile, "admission") == Some("draft_for_public_comment_not_binding_law")
        {
            if profile
                .get("effective_from")
                .is_some_and(|value| !value.is_null())
            {
                errors.push(format!(
                    "profiles[{index}]: draft regulation cannot have asserted effective date"
                ));
            }
        }
        if profile
            .get("cooling_off_seconds")
            .is_some_and(|value| !value.is_null())
        {
            errors.push(format!(
                "profiles[{index}]: no unsourced cooling-off interval may be asserted"
            ));
        }
    }

    for (case_index, case) in scenarios.iter().enumerate() {
        let profile_id = str_at(case, "profile_id").unwrap_or("");
        let Some(profile) = profile_map.get(profile_id) else {
            errors.push(format!(
                "scenarios[{case_index}]: unresolved profile {profile_id}"
            ));
            continue;
        };
        let alias = &case["alias"];
        let alias_id = str_at(alias, "alias_id").unwrap_or("");
        let generation = u64_at(alias, "generation").unwrap_or(0);
        let beneficiary = str_at(alias, "beneficiary_commitment").unwrap_or("");
        if alias_id.is_empty()
            || generation == 0
            || !beneficiary.starts_with("sha256:")
            || beneficiary.len() != 71
        {
            errors.push(format!(
                "scenarios[{case_index}]: invalid alias identity/generation/beneficiary commitment"
            ));
        }

        let events = array_at(case, "events").cloned().unwrap_or_default();
        let mut ids = BTreeSet::new();
        let mut previous_sequence = 0_u64;
        for (event_index, event) in events.iter().enumerate() {
            let event_id = str_at(event, "event_id").unwrap_or("");
            let sequence = u64_at(event, "sequence").unwrap_or(0);
            if event_id.is_empty() || !ids.insert(event_id.to_owned()) {
                errors.push(format!(
                    "scenarios[{case_index}].events[{event_index}]: missing or duplicate event id"
                ));
            }
            if sequence <= previous_sequence {
                errors.push(format!(
                    "scenarios[{case_index}].events[{event_index}]: event sequence not strictly increasing"
                ));
            }
            previous_sequence = sequence;
            if str_at(event, "event_type") == Some("alias_enumeration_attempt")
                && (!has_evidence(event, "rate_limit_exceeded")
                    || !has_evidence(event, "no_account_disclosure"))
            {
                errors.push(format!(
                    "scenarios[{case_index}]: alias enumeration event lacks anti-disclosure evidence"
                ));
            }
            if str_at(event, "event_type") == Some("payment_authorisation_recorded")
                && str_at(event, "authority_state") != Some("valid")
            {
                errors.push(format!(
                    "scenarios[{case_index}]: historical payment authorization lacks authority evidence"
                ));
            }
        }

        let (computed, resolution_allowed, requires_consent) = compute_state(case, profile, as_of);
        if str_at(case, "expected_state") != Some(computed) {
            errors.push(format!(
                "scenarios[{case_index}]: expected state does not match evidence-derived state {computed}"
            ));
        }
        if case.get("resolution_allowed").and_then(Value::as_bool) != Some(resolution_allowed) {
            errors.push(format!(
                "scenarios[{case_index}]: resolution_allowed does not match evidence-derived state"
            ));
        }
        if case
            .get("requires_new_payer_consent")
            .and_then(Value::as_bool)
            != Some(requires_consent)
        {
            errors.push(format!(
                "scenarios[{case_index}]: consent invalidation flag does not match evidence"
            ));
        }
        if case.get("dispatch_allowed").and_then(Value::as_bool) != Some(false) {
            errors.push(format!(
                "scenarios[{case_index}]: dispatch must remain disabled"
            ));
        }
        if computed == "active_resolved" {
            let alias_namespace = str_at(alias, "namespace").unwrap_or("");
            if alias_namespace.is_empty() {
                errors.push(format!(
                    "scenarios[{case_index}]: active alias lacks namespace"
                ));
            }
        }
    }

    errors
}

fn mutation_campaign(corpus: &Value) -> Result<usize, String> {
    if !semantic_errors(corpus).is_empty() {
        return Err("baseline corpus must pass before mutation campaign".into());
    }
    let mutations: [(&str, fn(&mut Value)); 12] = [
        ("enable external effects", |value| {
            value["external_effects_enabled"] = json!(true);
        }),
        ("enable dispatch", |value| {
            value["dispatch_enabled"] = json!(true);
        }),
        ("old alias after phone change marked active", |value| {
            value["scenarios"][1]["expected_state"] = json!("active_resolved");
            value["scenarios"][1]["resolution_allowed"] = json!(true);
        }),
        (
            "reassignment quarantine removed but still expected",
            |value| {
                value["scenarios"][5]["events"]
                    .as_array_mut()
                    .unwrap()
                    .retain(|event| str_at(event, "event_type") != Some("number_reassigned"));
            },
        ),
        ("SIM swap signal erased but quarantine retained", |value| {
            value["scenarios"][14]["events"]
                .as_array_mut()
                .unwrap()
                .retain(|event| str_at(event, "event_type") != Some("sim_swap_signal"));
        }),
        ("expired resolver promoted to active resolution", |value| {
            value["scenarios"][17]["expected_state"] = json!("active_resolved");
            value["scenarios"][17]["resolution_allowed"] = json!(true);
            value["scenarios"][17]["requires_new_payer_consent"] = json!(false);
        }),
        ("directory epoch advance erased", |value| {
            value["scenarios"][18]["events"]
                .as_array_mut()
                .unwrap()
                .retain(|event| str_at(event, "event_type") != Some("directory_epoch_advanced"));
        }),
        ("bank account closure erased", |value| {
            value["scenarios"][13]["events"]
                .as_array_mut()
                .unwrap()
                .retain(|event| str_at(event, "event_type") != Some("account_closed"));
        }),
        ("cross-bank default collision erased", |value| {
            value["scenarios"][11]["events"]
                .as_array_mut()
                .unwrap()
                .retain(|event| {
                    str_at(event, "event_type") != Some("cross_participant_binding_conflict")
                });
        }),
        (
            "new beneficiary under same binding fingerprint accepted",
            |value| {
                value["scenarios"][29]["expected_state"] = json!("active_resolved");
                value["scenarios"][29]["resolution_allowed"] = json!(true);
            },
        ),
        ("display mismatch retains old consent", |value| {
            value["scenarios"][26]["expected_state"] = json!("active_resolved");
            value["scenarios"][26]["resolution_allowed"] = json!(true);
            value["scenarios"][26]["requires_new_payer_consent"] = json!(false);
        }),
        ("draft proposal treated as final", |value| {
            value["scenarios"][15]["expected_state"] = json!("active_resolved");
            value["scenarios"][15]["resolution_allowed"] = json!(true);
        }),
    ];
    for (name, mutate) in mutations {
        let mut changed = corpus.clone();
        mutate(&mut changed);
        if semantic_errors(&changed).is_empty() {
            return Err(format!("negative control not rejected: {name}"));
        }
    }
    Ok(mutations.len())
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut corpus_text = DEFAULT_CORPUS.to_owned();
    let mut schema_text = DEFAULT_SCHEMA.to_owned();
    let mut args = std::env::args().skip(1);
    let mut self_test = false;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--corpus" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, "--corpus requires path")
                })?);
                corpus_text = fs::read_to_string(path)?;
            }
            "--schema" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, "--schema requires path")
                })?);
                schema_text = fs::read_to_string(path)?;
            }
            "--self-test" => self_test = true,
            "--help" | "-h" => {
                println!(
                    "sa-shapid-alias-lifecycle-evaluator [--corpus PATH] [--schema PATH] [--self-test]"
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
    let (_schema, corpus) = schema_errors(&schema_text, &corpus_text).map_err(|errors| {
        std::io::Error::new(std::io::ErrorKind::InvalidData, errors.join("; "))
    })?;
    let errors = semantic_errors(&corpus);
    let mutation_count = if self_test {
        mutation_campaign(&corpus)?
    } else {
        0
    };
    let report = json!({
        "evaluator":"mycelix.sa-shapid-alias-lifecycle.v1",
        "status":if errors.is_empty() {"PASS"} else {"FAIL"},
        "sources":array_at(&corpus,"source_register").map_or(0,Vec::len),
        "profiles":array_at(&corpus,"profiles").map_or(0,Vec::len),
        "scenarios":array_at(&corpus,"scenarios").map_or(0,Vec::len),
        "negative_mutations_rejected":mutation_count,
        "external_effects_enabled":false,
        "dispatch_enabled":false,
        "claim_ceiling":"synthetic alias lifecycle and currentness only"
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
    fn corpus_and_schema_validate() {
        let (_, corpus) = schema_errors(DEFAULT_SCHEMA, DEFAULT_CORPUS).expect("schema");
        let errors = semantic_errors(&corpus);
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(array_at(&corpus, "source_register").unwrap().len(), 5);
        assert_eq!(array_at(&corpus, "profiles").unwrap().len(), 5);
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 30);
    }

    #[test]
    fn mutation_campaign_rejects_all_12_controls() {
        let (_, corpus) = schema_errors(DEFAULT_SCHEMA, DEFAULT_CORPUS).expect("schema");
        assert_eq!(mutation_campaign(&corpus).expect("mutations"), 12);
    }
}
