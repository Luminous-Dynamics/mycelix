use jsonschema::draft202012;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const REASONS: [&str; 10] = [
    "actor_role_or_authority_conflict",
    "legal_hold_blocks_deletion",
    "secondary_purpose_unreviewed",
    "support_access_location_unknown",
    "required_fact_after_knowledge_frontier",
    "uk_regime_profile_not_qualified",
    "onward_transfer_mechanism_scope_incomplete",
    "onward_transfer_destination_unqualified",
    "accounting_profile_cannot_authorise_data_transfer",
    "foreign_support_access_without_transfer_qualification",
    "foreign_processor_transfer_unqualified",
];

fn main() {
    if let Err(error) = run() {
        eprintln!("mycelix-finance-data-flow-review: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut data_dir = root.join("../data-governance-v1");
    let mut output_path: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--data-dir" => {
                data_dir = PathBuf::from(args.next().ok_or("--data-dir requires a directory")?);
            }
            "--output" => {
                output_path = Some(PathBuf::from(args.next().ok_or("--output requires a file path")?));
            }
            "--help" | "-h" => {
                println!(
                    "Mycelix Finance Data-Flow Review Evaluator v1.0.0\n\n\
                     --data-dir PATH  Context artifact directory (default: sibling data-governance-v1)\n\
                     --output PATH    Write the deterministic report to PATH instead of stdout\n\
                     --help           Show this help\n\n\
                     This evaluator emits review triage only; it never authorises transfers or deletion."
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}; use --help").into()),
        }
    }

    let schema: Value = read_json(data_dir.join("finance_data_flow_context_v1.0.0.schema.json"))?;
    let candidates_path = data_dir.join("fixtures/finance_data_flow_candidates_v1.0.0.json");
    let candidates_bytes = fs::read(&candidates_path)?;
    let candidates: Value = serde_json::from_slice(&candidates_bytes)?;
    let oracle_schema: Value = read_json(data_dir.join("finance_data_flow_oracle_v1.0.0.schema.json"))?;
    let oracle: Value = read_json(data_dir.join("fixtures/finance_data_flow_oracle_v1.0.0.json"))?;
    let report_schema: Value = read_json(data_dir.join("finance_data_flow_review_report_v1.0.0.schema.json"))?;

    validate_instance("data-flow context bundle", &schema, &candidates)?;
    validate_instance("data-flow public oracle", &oracle_schema, &oracle)?;

    let mut results = Vec::new();
    for context in array(&candidates, "contexts")? {
        results.push(evaluate_context(context)?);
    }

    let report = json!({
        "report_id": "MYCELIX-FINANCE-DATA-FLOW-REVIEW-REPORT-V1",
        "schema_version": "1.0.0",
        "evaluator_id": "MYCELIX-FINANCE-DATA-FLOW-REVIEW-V1",
        "claim_ceiling": "synthetic_review_triage_only_not_transfer_authorization_or_compliance",
        "source_bundle_sha256": hex::encode(Sha256::digest(&candidates_bytes)),
        "results": results,
    });

    validate_instance("data-flow review report", &report_schema, &report)?;
    validate_oracle_conformance(&report, &oracle)?;

    let rendered = serde_json::to_string_pretty(&report)?;
    if let Some(path) = output_path {
        fs::write(path, format!("{rendered}\n"))?;
        println!(
            "PASS: {} synthetic contexts deterministically match the public review oracle; report written. No transfer, compliance, or deletion authority is produced.",
            array(&report, "results")?.len()
        );
    } else {
        println!("{rendered}");
    }
    Ok(())
}

fn read_json(path: PathBuf) -> Result<Value, Box<dyn Error>> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}

fn validate_instance(label: &str, schema: &Value, instance: &Value) -> Result<(), String> {
    draft202012::meta::validate(schema)
        .map_err(|error| format!("{label}: invalid Draft 2020-12 schema: {error}"))?;
    let validator = draft202012::options()
        .should_validate_formats(true)
        .build(schema)
        .map_err(|error| format!("{label}: could not compile Draft 2020-12 schema: {error}"))?;
    let errors = validator
        .iter_errors(instance)
        .map(|error| format!("{} at {}", error, error.instance_path()))
        .collect::<Vec<_>>();
    if errors.is_empty() {
        Ok(())
    } else {
        Err(format!("{label}: {}", errors.join("; ")))
    }
}

fn array<'a>(value: &'a Value, key: &str) -> Result<&'a [Value], String> {
    value
        .get(key)
        .and_then(Value::as_array)
        .map(Vec::as_slice)
        .ok_or_else(|| format!("missing or invalid array: {key}"))
}

fn string<'a>(value: &'a Value, key: &str) -> Result<&'a str, String> {
    value
        .get(key)
        .and_then(Value::as_str)
        .ok_or_else(|| format!("missing or invalid string: {key}"))
}

fn optional_string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn find_by_ref<'a>(values: &'a [Value], key: &str, wanted: &str) -> Option<&'a Value> {
    values
        .iter()
        .find(|value| optional_string(value, key) == Some(wanted))
}

fn interval_overlaps(
    valid_from: &str,
    valid_to: Option<&str>,
    period_start: &str,
    period_end: &str,
) -> bool {
    valid_from <= period_end && valid_to.is_none_or(|end| end >= period_start)
}

fn is_active(value: &Value, from_key: &str, to_key: &str, period: &Value) -> bool {
    let Some(from) = optional_string(value, from_key) else {
        return false;
    };
    let Some(start) = optional_string(period, "start") else {
        return false;
    };
    let Some(end) = optional_string(period, "end") else {
        return false;
    };
    interval_overlaps(from, optional_string(value, to_key), start, end)
}

fn is_known_at(value: &Value, context: &Value) -> bool {
    let Some(recorded_at) = optional_string(value, "recorded_at") else {
        return false;
    };
    let Some(knowledge_at) = optional_string(context, "knowledge_at") else {
        return false;
    };
    recorded_at <= knowledge_at
}

fn actor_has_role(actor: &Value, role: &str) -> bool {
    actor
        .get("actor_roles")
        .and_then(Value::as_array)
        .is_some_and(|roles| roles.iter().any(|value| value.as_str() == Some(role)))
}

fn push_reason(reasons: &mut BTreeSet<String>, reason: &str) {
    reasons.insert(reason.to_owned());
}

fn controller_scopes(context: &Value) -> Result<BTreeSet<String>, String> {
    let mut scopes = BTreeSet::new();
    for actor in array(context, "actors")? {
        if actor_has_role(actor, "controller") || actor_has_role(actor, "responsible_party") {
            if let Some(scope) = optional_string(actor, "home_scope_ref") {
                scopes.insert(scope.to_owned());
            }
        }
    }
    Ok(scopes)
}

fn mechanism_qualifies_for_fixture_scope(
    context: &Value,
    flow: &Value,
    destination_scope: &str,
) -> Result<bool, String> {
    let period = context.get("period").ok_or("context missing period")?;
    let mechanisms = array(context, "transfer_mechanisms")?;
    let profiles = array(context, "legal_profiles")?;
    let flow_profile_refs = array(flow, "legal_profile_refs")?;
    for mechanism_ref in array(flow, "transfer_mechanism_refs")? {
        let Some(mechanism_id) = mechanism_ref.as_str() else {
            return Err("transfer_mechanism_refs must contain strings".to_owned());
        };
        let Some(mechanism) = find_by_ref(mechanisms, "mechanism_ref", mechanism_id) else {
            continue;
        };
        if optional_string(mechanism, "destination_scope_ref") != Some(destination_scope)
            || optional_string(mechanism, "status") != Some("synthetic_approved_fixture_only")
            || array(mechanism, "evidence_refs")?.is_empty()
            || array(mechanism, "reviewer_refs")?.is_empty()
            || !is_active(mechanism, "valid_from", "valid_to", period)
            || !is_known_at(mechanism, context)
        {
            continue;
        }
        let Some(profile_ref) = optional_string(mechanism, "profile_ref") else {
            continue;
        };
        if !flow_profile_refs.iter().any(|value| value.as_str() == Some(profile_ref)) {
            continue;
        }
        let Some(profile) = find_by_ref(profiles, "profile_ref", profile_ref) else {
            continue;
        };
        if optional_string(profile, "status") == Some("synthetic_unqualified")
            || !is_active(profile, "effective_from", "effective_to", period)
            || !is_known_at(profile, context)
        {
            continue;
        }
        // "synthetic_approved_fixture_only" is a closed-world benchmark token, not a legal approval.
        return Ok(true);
    }
    Ok(false)
}

fn foreign_locations<'a>(
    context: &Value,
    flow: &'a Value,
    local_scopes: &BTreeSet<String>,
) -> Result<Vec<(&'a str, &'a str)>, String> {
    let period = context.get("period").ok_or("context missing period")?;
    let mut locations = Vec::new();
    for location in array(flow, "location_observations")? {
        if !is_active(location, "valid_from", "valid_to", period)
            || !is_known_at(location, context)
        {
            continue;
        }
        if optional_string(location, "assertion_state") == Some("unknown") {
            continue;
        }
        let Some(scope) = optional_string(location, "scope_ref") else {
            continue;
        };
        if local_scopes.contains(scope) {
            continue;
        }
        let facet = string(location, "facet")?;
        locations.push((facet, scope));
    }
    Ok(locations)
}

fn evaluate_context(context: &Value) -> Result<Value, String> {
    let context_id = string(context, "context_id")?;
    let mut reasons = BTreeSet::new();
    let local_scopes = controller_scopes(context)?;
    if local_scopes.len() != 1 {
        push_reason(&mut reasons, "actor_role_or_authority_conflict");
    }
    let local_scope_set = local_scopes.clone();

    // Conflicting or disputed role assertions prevent a clean actor/authority interpretation.
    for claim in array(context, "actor_role_claims")? {
        if !is_known_at(claim, context) {
            push_reason(&mut reasons, "required_fact_after_knowledge_frontier");
            continue;
        }
        if optional_string(claim, "assertion_state") == Some("synthetic_disputed")
            || !array(claim, "conflicts_with_claim_refs")?.is_empty()
        {
            push_reason(&mut reasons, "actor_role_or_authority_conflict");
        }
    }

    // A legal hold is a hard stop on deletion requests; this evaluator never performs deletion.
    for retention in array(context, "retention_policies")? {
        if !is_known_at(retention, context) {
            push_reason(&mut reasons, "required_fact_after_knowledge_frontier");
            continue;
        }
        let deletion_requested = matches!(
            optional_string(retention, "deletion_state"),
            Some("requested" | "executed")
        );
        if optional_string(retention, "legal_hold_state") == Some("active") && deletion_requested {
            push_reason(&mut reasons, "legal_hold_blocks_deletion");
        }
    }

    let purposes = array(context, "purposes")?;
    for flow in array(context, "flows")? {
        if !is_known_at(flow, context) {
            push_reason(&mut reasons, "required_fact_after_knowledge_frontier");
            continue;
        }
        let purpose_ref = string(flow, "purpose_ref")?;
        if let Some(purpose) = find_by_ref(purposes, "purpose_ref", purpose_ref) {
            if !is_known_at(purpose, context) {
                push_reason(&mut reasons, "required_fact_after_knowledge_frontier");
            } else if matches!(
                optional_string(purpose, "status"),
                Some("synthetic_unreviewed" | "synthetic_disputed")
            ) && is_active(purpose, "valid_from", "valid_to", context.get("period").ok_or("context missing period")?)
            {
                push_reason(&mut reasons, "secondary_purpose_unreviewed");
            }
        }

        let flow_locations = array(flow, "location_observations")?;
        for location in flow_locations {
            if !is_known_at(location, context) {
                push_reason(&mut reasons, "required_fact_after_knowledge_frontier");
                continue;
            }
            if optional_string(location, "facet") == Some("support_access")
                && optional_string(location, "assertion_state") == Some("unknown")
                && is_active(location, "valid_from", "valid_to", context.get("period").ok_or("context missing period")?)
            {
                push_reason(&mut reasons, "support_access_location_unknown");
            }
        }

        let foreign = foreign_locations(context, flow, &local_scope_set)?;
        if foreign.is_empty() {
            continue;
        }

        // The UK fixture has a separate, effective-dated regime profile. A candidate/reference
        // profile is not a qualified review profile and cannot silently fall back to EU GDPR.
        let profile_refs = array(flow, "legal_profile_refs")?;
        let profiles = array(context, "legal_profiles")?;
        for profile_ref in profile_refs {
            let Some(profile_id) = profile_ref.as_str() else {
                return Err("legal_profile_refs must contain strings".to_owned());
            };
            if let Some(profile) = find_by_ref(profiles, "profile_ref", profile_id) {
                if !is_known_at(profile, context) {
                    push_reason(&mut reasons, "required_fact_after_knowledge_frontier");
                    continue;
                }
                let regime = optional_string(profile, "regime_id").unwrap_or_default();
                if regime.contains("UK")
                    && optional_string(profile, "status") != Some("synthetic_approved_fixture_only")
                    && is_active(profile, "effective_from", "effective_to", context.get("period").ok_or("context missing period")?)
                {
                    push_reason(&mut reasons, "uk_regime_profile_not_qualified");
                }
            }
        }

        let onward = foreign
            .iter()
            .filter(|(facet, _)| *facet == "onward_recipient")
            .map(|(_, scope)| *scope)
            .collect::<BTreeSet<_>>();
        if !onward.is_empty() {
            let mechanisms = array(context, "transfer_mechanisms")?;
            let has_any_referenced_mechanism = !array(flow, "transfer_mechanism_refs")?.is_empty();
            let mut all_destinations_covered = true;
            for destination in &onward {
                if !mechanism_qualifies_for_fixture_scope(context, flow, destination)? {
                    all_destinations_covered = false;
                }
            }
            if has_any_referenced_mechanism || onward.len() > 1 {
                if !all_destinations_covered {
                    push_reason(&mut reasons, "onward_transfer_mechanism_scope_incomplete");
                }
            } else if mechanisms.is_empty() || !all_destinations_covered {
                push_reason(&mut reasons, "onward_transfer_destination_unqualified");
            }
        }

        // An accounting-only profile is never a data-transfer profile. Keep this explicit and
        // indeterminate rather than mapping it to a transfer-permission failure.
        let referenced_profiles = profile_refs
            .iter()
            .filter_map(Value::as_str)
            .filter_map(|id| find_by_ref(profiles, "profile_ref", id))
            .collect::<Vec<_>>();
        let has_accounting_profile = referenced_profiles.iter().any(|profile| {
            optional_string(profile, "regime_id")
                .is_some_and(|regime| regime.contains("ACCOUNTING"))
        });
        let has_non_accounting_profile = referenced_profiles.iter().any(|profile| {
            optional_string(profile, "regime_id")
                .is_some_and(|regime| !regime.contains("ACCOUNTING"))
        });
        if has_accounting_profile
            && !has_non_accounting_profile
            && array(flow, "transfer_mechanism_refs")?.is_empty()
        {
            push_reason(&mut reasons, "accounting_profile_cannot_authorise_data_transfer");
            continue;
        }

        for (facet, destination) in foreign {
            if mechanism_qualifies_for_fixture_scope(context, flow, destination)? {
                continue;
            }
            match facet {
                "support_access" if string(flow, "operation")? == "support_access" => {
                    push_reason(&mut reasons, "foreign_support_access_without_transfer_qualification");
                }
                "onward_recipient" => {
                    push_reason(&mut reasons, "onward_transfer_destination_unqualified");
                }
                "primary_storage" | "active_processing"
                    if string(flow, "operation")? != "onward_transfer" =>
                {
                    let recipient_ref = string(flow, "recipient_actor_ref")?;
                    let recipient = find_by_ref(array(context, "actors")?, "actor_ref", recipient_ref);
                    if recipient.is_some_and(|actor| {
                        actor_has_role(actor, "processor") || actor_has_role(actor, "operator")
                    }) || facet == "active_processing"
                    {
                        push_reason(&mut reasons, "foreign_processor_transfer_unqualified");
                    }
                }
                "backup_dr" if string(flow, "operation")? == "backup" => {
                    push_reason(&mut reasons, "onward_transfer_destination_unqualified");
                }
                "recipient_establishment" if string(flow, "operation")? == "disclosure" => {
                    push_reason(&mut reasons, "foreign_processor_transfer_unqualified");
                }
                _ => {}
            }
        }
    }

    let primary_reason = REASONS.iter().find(|reason| reasons.contains(**reason)).copied();
    let disposition = match primary_reason {
        Some("accounting_profile_cannot_authorise_data_transfer")
        | Some("support_access_location_unknown")
        | Some("required_fact_after_knowledge_frontier") => "indeterminate",
        Some(_) => "needs_review",
        None => "no_trigger_detected",
    };
    let ordered_reasons = REASONS
        .iter()
        .filter(|reason| reasons.contains(**reason))
        .copied()
        .collect::<Vec<_>>();
    Ok(json!({
        "context_id": context_id,
        "review_disposition": disposition,
        "primary_reason_code": primary_reason,
        "triggered_reason_codes": ordered_reasons,
    }))
}

fn validate_oracle_conformance(report: &Value, oracle: &Value) -> Result<(), String> {
    let results = array(report, "results")?;
    let expectations = array(oracle, "expectations")?;
    if results.len() != expectations.len() {
        return Err(format!(
            "public oracle count {} does not match evaluated result count {}",
            expectations.len(),
            results.len()
        ));
    }
    for expectation in expectations {
        let context_id = string(expectation, "context_id")?;
        let Some(result) = results
            .iter()
            .find(|result| optional_string(result, "context_id") == Some(context_id))
        else {
            return Err(format!("missing evaluated context {context_id}"));
        };
        let expected_disposition = string(expectation, "expected_review_disposition")?;
        let expected_reason = string(expectation, "reason_code")?;
        if optional_string(result, "review_disposition") != Some(expected_disposition)
            || optional_string(result, "primary_reason_code") != Some(expected_reason)
        {
            return Err(format!(
                "{context_id}: expected {expected_disposition}/{expected_reason}, got {}/{}",
                optional_string(result, "review_disposition").unwrap_or("<missing>"),
                optional_string(result, "primary_reason_code").unwrap_or("<null-or-missing>")
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (Value, Value, Value) {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../data-governance-v1");
        let candidates: Value = read_json(root.join("fixtures/finance_data_flow_candidates_v1.0.0.json")).unwrap();
        let oracle: Value = read_json(root.join("fixtures/finance_data_flow_oracle_v1.0.0.json")).unwrap();
        let context_schema: Value = read_json(root.join("finance_data_flow_context_v1.0.0.schema.json")).unwrap();
        (candidates, oracle, context_schema)
    }

    fn context_index(bundle: &Value, id: &str) -> usize {
        array(bundle, "contexts").unwrap().iter()
            .position(|item| optional_string(item, "context_id") == Some(id)).unwrap()
    }

    #[test]
    fn all_ten_public_expectations_match_the_evaluator() {
        let (candidates, oracle, _) = fixtures();
        let results = array(&candidates, "contexts").unwrap().iter()
            .map(evaluate_context).collect::<Result<Vec<_>, _>>().unwrap();
        let result_count = results.len();
        let report = json!({ "results": results });
        validate_oracle_conformance(&report, &oracle).unwrap();
        assert_eq!(result_count, 10);
    }

    #[test]
    fn local_only_negative_control_does_not_trigger_cross_border_review() {
        let (mut candidates, _, _) = fixtures();
        let index = context_index(&candidates, "DG-001-FOREIGN-SUPPORT");
        let context = &mut candidates["contexts"][index];
        context["flows"].as_array_mut().unwrap().retain(|flow| {
            optional_string(flow, "operation") != Some("support_access")
        });
        let result = evaluate_context(context).unwrap();
        assert_eq!(result["review_disposition"], "no_trigger_detected");
        assert!(result["triggered_reason_codes"].as_array().unwrap().is_empty());
    }

    #[test]
    fn unassessed_mechanism_is_not_treated_as_fixture_qualification() {
        let (candidates, _, _) = fixtures();
        let context = &candidates["contexts"][context_index(&candidates, "DG-001-FOREIGN-SUPPORT")];
        let result = evaluate_context(context).unwrap();
        assert_eq!(result["primary_reason_code"], "foreign_support_access_without_transfer_qualification");
        assert_eq!(result["review_disposition"], "needs_review");
    }

    #[test]
    fn post_2026_uk_profile_is_not_fallback_resolved_to_eu_profile() {
        let (candidates, _, _) = fixtures();
        let context = &candidates["contexts"][context_index(&candidates, "DG-005-UK-REGIME-VERSION")];
        let result = evaluate_context(context).unwrap();
        assert_eq!(result["primary_reason_code"], "uk_regime_profile_not_qualified");
        assert_eq!(result["review_disposition"], "needs_review");
    }

    #[test]
    fn multiple_onward_destinations_require_independent_mechanism_scope() {
        let (candidates, _, _) = fixtures();
        let context = &candidates["contexts"][context_index(&candidates, "DG-004-EU-ONWARD")];
        let result = evaluate_context(context).unwrap();
        assert_eq!(result["primary_reason_code"], "onward_transfer_mechanism_scope_incomplete");
        assert_eq!(result["review_disposition"], "needs_review");
    }

    #[test]
    fn accounting_profile_is_never_transfer_authority() {
        let (candidates, _, _) = fixtures();
        let context = &candidates["contexts"][context_index(&candidates, "DG-006-ACCOUNTING-ONLY")];
        let result = evaluate_context(context).unwrap();
        assert_eq!(result["primary_reason_code"], "accounting_profile_cannot_authorise_data_transfer");
        assert_eq!(result["review_disposition"], "indeterminate");
    }

    #[test]
    fn unknown_support_access_is_indeterminate_not_local() {
        let (candidates, _, _) = fixtures();
        let context = &candidates["contexts"][context_index(&candidates, "DG-009-UNKNOWN-REMOTE-ACCESS")];
        let result = evaluate_context(context).unwrap();
        assert_eq!(result["primary_reason_code"], "support_access_location_unknown");
        assert_eq!(result["review_disposition"], "indeterminate");
    }

    #[test]
    fn facts_recorded_after_knowledge_frontier_are_excluded_and_indeterminate() {
        let (mut candidates, _, _) = fixtures();
        let index = context_index(&candidates, "DG-001-FOREIGN-SUPPORT");
        let context = &mut candidates["contexts"][index];
        let support_flow = context["flows"].as_array_mut().unwrap()
            .iter_mut().find(|flow| optional_string(flow, "operation") == Some("support_access")).unwrap();
        let support_location = support_flow["location_observations"].as_array_mut().unwrap()
            .iter_mut().find(|location| optional_string(location, "facet") == Some("support_access")).unwrap();
        support_location["recorded_at"] = Value::String("2026-11-05T10:45:00Z".to_owned());
        let result = evaluate_context(context).unwrap();
        assert_eq!(result["primary_reason_code"], "required_fact_after_knowledge_frontier");
        assert_eq!(result["review_disposition"], "indeterminate");
        assert!(!array(&result, "triggered_reason_codes").unwrap().iter()
            .any(|code| code.as_str() == Some("foreign_support_access_without_transfer_qualification")));
    }

    #[test]
    fn active_legal_hold_remains_a_review_blocker() {
        let (candidates, _, _) = fixtures();
        let context = &candidates["contexts"][context_index(&candidates, "DG-007-RETENTION-LEGAL-HOLD")];
        let result = evaluate_context(context).unwrap();
        assert_eq!(result["primary_reason_code"], "legal_hold_blocks_deletion");
        assert_eq!(result["review_disposition"], "needs_review");
    }

    #[test]
    fn disputed_actor_authority_is_not_resolved_by_actor_home_metadata() {
        let (candidates, _, _) = fixtures();
        let context = &candidates["contexts"][context_index(&candidates, "DG-010-ACTOR-ROLE-CONFLICT")];
        let result = evaluate_context(context).unwrap();
        assert_eq!(result["primary_reason_code"], "actor_role_or_authority_conflict");
        assert_eq!(result["review_disposition"], "needs_review");
    }
}
