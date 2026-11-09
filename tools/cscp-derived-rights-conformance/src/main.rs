use std::{env, fs, process::ExitCode};

use serde_json::Value;

const FIXTURE: &str =
    include_str!("../../../specs/creative-assets/examples/derived-rights-conformance-vectors.json");

#[derive(Debug, Clone, PartialEq, Eq)]
struct Decision {
    decision: &'static str,
    reason_code: &'static str,
}

fn decision(decision: &'static str, reason_code: &'static str) -> Decision {
    Decision {
        decision,
        reason_code,
    }
}

fn string_at<'a>(value: &'a Value, path: &[&str]) -> Option<&'a str> {
    let mut current = value;
    for key in path {
        current = current.get(*key)?;
    }
    current.as_str()
}

fn operation_class(operation: &str) -> &str {
    match operation {
        "thumbnail" | "crop" | "resize" | "normalize" | "transcode" | "ocr" | "transcribe"
        | "caption" | "segment" => "transform",
        _ => operation,
    }
}

fn action_matches(action: &str, operation: &str) -> bool {
    action == operation || action == operation_class(operation)
}

fn list_contains_action(value: Option<&Value>, operation: &str) -> bool {
    value
        .and_then(Value::as_array)
        .is_some_and(|items| {
            items
                .iter()
                .filter_map(Value::as_str)
                .any(|action| action_matches(action, operation))
        })
}

fn permission_matches(
    permission: &Value,
    operation: &str,
    actor: &str,
    purpose: Option<&str>,
    at: Option<&str>,
) -> bool {
    if let Some(action) = permission.as_str() {
        return action_matches(action, operation);
    }

    let Some(granted_operation) = permission.get("operation").and_then(Value::as_str) else {
        return false;
    };
    if !action_matches(granted_operation, operation) {
        return false;
    }

    if permission.get("actor").and_then(Value::as_str) != Some(actor) {
        return false;
    }

    if let Some(granted_purpose) = permission.get("purpose").and_then(Value::as_str) {
        if purpose != Some(granted_purpose) {
            return false;
        }
    }

    let valid_from = permission.get("validFrom").and_then(Value::as_str);
    let valid_until = permission.get("validUntil").and_then(Value::as_str);
    match (valid_from, valid_until, at) {
        (Some(start), Some(end), Some(now)) => start <= now && now <= end,
        (None, None, _) => true,
        _ => false,
    }
}

fn community_permission_is_scoped(source: &Value) -> bool {
    string_at(source, &["communityGovernance", "capacityStatus"])
        == Some("verified-for-this-scope")
        && string_at(source, &["communityGovernance", "authorityReference"]).is_some()
        && string_at(source, &["communityGovernance", "processReference"]).is_some()
        && string_at(source, &["communityGovernance", "decisionReference"]).is_some()
}

fn valid_supersession<'a>(
    case: &'a Value,
    sources: &'a [Value],
    operation: &str,
    actor: &str,
    purpose: Option<&str>,
    at: Option<&str>,
) -> Option<&'a str> {
    let policy = case.get("policySupersession")?;
    let source_resource = policy.get("sourceResource")?.as_str()?;
    let output_resource = policy.get("outputResource")?.as_str()?;
    let derivation_output = string_at(case, &["derivation", "output"])?;
    let derivation_parents = case
        .get("derivation")?
        .get("parents")?
        .as_array()?;
    let grantee = policy.get("grantee")?.as_str()?;
    let granted_purpose = policy.get("purpose")?.as_str()?;
    let granted_jurisdiction = policy.get("jurisdiction")?.as_str()?;
    let granted_channel = policy.get("channel")?.as_str()?;
    let requested_jurisdiction = case.get("jurisdiction")?.as_str()?;
    let requested_channel = case.get("channel")?.as_str()?;
    let valid_from = policy.get("validFrom")?.as_str()?;
    let valid_until = policy.get("validUntil")?.as_str()?;
    let now = at?;
    let authority = policy.get("authority")?.as_str()?;
    let _evidence = policy.get("evidence")?.as_str()?;
    let _approval_event = policy.get("approvalEvent")?.as_str()?;
    let prior_version = policy.get("priorPolicyVersion")?.as_str()?;
    let current_version = policy.get("currentPolicyVersion")?.as_str()?;

    let exact_scope = output_resource == derivation_output
        && derivation_parents
            .iter()
            .filter_map(Value::as_str)
            .any(|parent| parent == source_resource)
        && policy.get("operation").and_then(Value::as_str) == Some(operation)
        && grantee == actor
        && purpose == Some(granted_purpose)
        && requested_jurisdiction == granted_jurisdiction
        && requested_channel == granted_channel
        && policy.get("authorityReference").and_then(Value::as_str).is_some()
        && policy.get("authorityRole").and_then(Value::as_str).is_some()
        && policy.get("decisionReference").and_then(Value::as_str).is_some()
        && policy.get("supersededRuleId").and_then(Value::as_str).is_some()
        && authority == "verified-for-exact-scope"
        && prior_version != current_version
        && valid_from <= now
        && now <= valid_until;

    if !exact_scope {
        return None;
    }

    let source_has_superseded_prohibition = sources.iter().any(|source| {
        source.get("resource").and_then(Value::as_str) == Some(source_resource)
            && list_contains_action(source.get("prohibitions"), operation)
    });

    source_has_superseded_prohibition.then_some(source_resource)
}

fn evaluate_case(case: &Value) -> Decision {
    let Some(operation) = case.get("operation").and_then(Value::as_str) else {
        return decision("indeterminate", "REQUEST_SCOPE_MISSING");
    };
    let Some(actor) = case.get("actor").and_then(Value::as_str) else {
        return decision("indeterminate", "REQUEST_SCOPE_MISSING");
    };
    let Some(sources) = case.get("source").and_then(Value::as_array) else {
        return decision("indeterminate", "SOURCE_LINEAGE_MISSING");
    };
    if sources.is_empty() {
        return decision("indeterminate", "SOURCE_LINEAGE_MISSING");
    }

    // A stale cache must not serve a previous permit after the policy epoch advances.
    if let Some(cache) = case.get("cache") {
        let cached = cache.get("cachedPolicyEpoch").and_then(Value::as_u64);
        let current = cache.get("sourcePolicyEpoch").and_then(Value::as_u64);
        if matches!((cached, current), (Some(old), Some(new)) if old < new) {
            return decision("deny", "STALE_POLICY_EPOCH");
        }
    }

    // Context failures are indeterminate and always deny at the enforcement boundary.
    for source in sources {
        if string_at(source, &["policyStatus"]) == Some("unavailable") {
            return decision("indeterminate", "POLICY_UNAVAILABLE");
        }
        if string_at(source, &["lineageStatus"]) == Some("incomplete") {
            return decision("indeterminate", "INCOMPLETE_DERIVATION_LINEAGE");
        }
        if string_at(source, &["grantorCapacity"]) == Some("unknown") {
            return decision("indeterminate", "GRANTOR_AUTHORITY_UNRESOLVED");
        }
        if string_at(source, &["grantStatus"]) == Some("expired") {
            return decision("deny", "GRANT_EXPIRED");
        }
        if string_at(source, &["grantStatus"]) == Some("revoked") {
            return decision("deny", "GRANT_REVOKED");
        }
        if string_at(source, &["policyConflictStrategy"]) == Some("invalid")
            && string_at(source, &["policyComposition"]) == Some("conflicting")
        {
            return decision("indeterminate", "POLICY_COMPOSITION_INVALID");
        }
        if let (Some(source_tenant), Some(query_tenant)) = (
            source.get("tenant").and_then(Value::as_str),
            string_at(case, &["query", "tenant"]),
        ) {
            if source_tenant != query_tenant {
                return decision("deny", "TENANT_SCOPE_MISMATCH");
            }
        }
    }

    if operation == "disclose-location"
        && case
            .get("protectedFields")
            .and_then(Value::as_array)
            .is_some_and(|fields| {
                fields.iter().any(|field| {
                    field.get("access").and_then(Value::as_str) == Some("sealed")
                        && field.get("field").and_then(Value::as_str) == Some("exactLocation")
                })
            })
    {
        return decision("deny", "FIELD_ACCESS_RESTRICTED");
    }

    let purpose = case.get("purpose").and_then(Value::as_str);
    let at = case.get("at").and_then(Value::as_str);
    let superseded_source = valid_supersession(case, sources, operation, actor, purpose, at);

    // Prohibitions are evaluated across every ancestor before positive grants.
    for source in sources {
        let resource = source.get("resource").and_then(Value::as_str);
        if superseded_source.is_some() && resource == superseded_source {
            continue;
        }
        if list_contains_action(source.get("prohibitions"), operation) {
            if list_contains_action(
                case.get("derivation").and_then(|d| d.get("outputPermissions")),
                operation,
            ) {
                return decision(
                    "deny",
                    "ANCESTOR_RESTRICTION_CANNOT_BE_WIDENED",
                );
            }
            return if sources.len() > 1 {
                decision("deny", "ANCESTOR_PROHIBITION")
            } else {
                decision("deny", "EXPLICIT_PROHIBITION")
            };
        }
    }

    // Every source ancestor must independently permit the requested operation.
    let mut community_scoped_grant = false;
    for source in sources {
        let resource = source.get("resource").and_then(Value::as_str);
        if superseded_source.is_some() && resource == superseded_source {
            continue;
        }

        let permissions = source.get("permissions").and_then(Value::as_array);
        let permitted = permissions.is_some_and(|items| {
            items.iter().any(|permission| {
                permission_matches(permission, operation, actor, purpose, at)
            })
        });
        if !permitted {
            return decision("deny", "NO_EXPLICIT_OPERATION_GRANT");
        }

        if source.get("communityGovernance").is_some()
            && community_permission_is_scoped(source)
        {
            community_scoped_grant = true;
        }
    }

    if superseded_source.is_some() {
        decision("permit", "AUTHORIZED_POLICY_SUPERSESSION")
    } else if community_scoped_grant {
        decision("permit", "COMMUNITY_AUTHORITY_SCOPED_GRANT")
    } else {
        decision("permit", "EXPLICIT_OPERATION_GRANT")
    }
}

fn validate_corpus(document: &Value) -> Result<usize, String> {
    if document.get("schemaVersion").and_then(Value::as_str) != Some("0.1.0") {
        return Err("unsupported or missing schemaVersion".into());
    }

    let cases = document
        .get("cases")
        .and_then(Value::as_array)
        .ok_or_else(|| "cases must be an array".to_owned())?;
    if cases.len() < 17 {
        return Err(format!("expected at least 17 cases, found {}", cases.len()));
    }

    let mut identifiers = std::collections::HashSet::new();
    for case in cases {
        let id = case
            .get("id")
            .and_then(Value::as_str)
            .ok_or_else(|| "case has no string id".to_owned())?;
        if !identifiers.insert(id) {
            return Err(format!("duplicate case id: {id}"));
        }
        let expected = case
            .get("expected")
            .ok_or_else(|| format!("{id}: missing expected object"))?;
        let expected_decision = expected
            .get("decision")
            .and_then(Value::as_str)
            .ok_or_else(|| format!("{id}: missing expected decision"))?;
        if !matches!(expected_decision, "permit" | "deny" | "indeterminate") {
            return Err(format!("{id}: unsupported expected decision"));
        }
        if expected_decision == "indeterminate"
            && expected.get("enforcementDisposition").and_then(Value::as_str) != Some("deny")
        {
            return Err(format!(
                "{id}: indeterminate must have enforcementDisposition=deny"
            ));
        }
        if expected_decision == "permit"
            && expected.get("scopeBound").and_then(Value::as_bool) == Some(false)
        {
            return Err(format!("{id}: permit cannot declare an unbound scope"));
        }
    }

    Ok(cases.len())
}

fn check_document(document: &Value) -> Result<usize, Vec<String>> {
    let count = validate_corpus(document).map_err(|error| vec![error])?;
    let cases = document
        .get("cases")
        .and_then(Value::as_array)
        .ok_or_else(|| vec!["cases must be an array".to_owned()])?;

    let mut errors = Vec::new();
    for case in cases {
        let id = case
            .get("id")
            .and_then(Value::as_str)
            .unwrap_or("<missing-id>");
        let actual = evaluate_case(case);
        let expected = &case["expected"];
        let expected_decision = expected["decision"].as_str().unwrap_or("");
        let expected_reason = expected["reasonCode"].as_str().unwrap_or("");
        if actual.decision != expected_decision || actual.reason_code != expected_reason {
            errors.push(format!(
                "{id}: expected {} / {}, got {} / {}",
                expected_decision, expected_reason, actual.decision, actual.reason_code
            ));
        }
    }

    if errors.is_empty() {
        Ok(count)
    } else {
        Err(errors)
    }
}

fn run(path: &str) -> ExitCode {
    let content = match fs::read_to_string(path) {
        Ok(content) => content,
        Err(error) => {
            eprintln!("input error: {path}: {error}");
            return ExitCode::from(2);
        }
    };
    let document: Value = match serde_json::from_str(&content) {
        Ok(document) => document,
        Err(error) => {
            eprintln!("invalid JSON: {path}: {error}");
            return ExitCode::from(2);
        }
    };

    match check_document(&document) {
        Ok(count) => {
            println!("PASS: {count} CSCP derived-rights vectors matched the Rust reference oracle");
            ExitCode::SUCCESS
        }
        Err(errors) => {
            eprintln!("FAIL: {} conformance error(s)", errors.len());
            for error in errors {
                eprintln!("- {error}");
            }
            ExitCode::FAILURE
        }
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    match args.as_slice() {
        [_, flag, path] if flag == "--check" => run(path),
        _ => {
            eprintln!(
                "usage: {} --check <derived-rights-conformance-vectors.json>",
                args.first().map(String::as_str).unwrap_or("cscp-derived-rights-conformance")
            );
            ExitCode::from(2)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Value {
        serde_json::from_str(FIXTURE).expect("frozen rights fixture is valid JSON")
    }

    #[test]
    fn all_frozen_vectors_match_the_reference_oracle() {
        let document = fixture();
        let result = check_document(&document);
        assert!(result.is_ok(), "{result:?}");
        assert_eq!(result.unwrap(), 17);
    }

    #[test]
    fn a_local_grant_cannot_override_an_ancestor_prohibition() {
        let mut document = fixture();
        let case = document["cases"]
            .as_array_mut()
            .expect("cases array")
            .iter_mut()
            .find(|case| case["id"] == "DR-012")
            .expect("supersession vector");
        case["policySupersession"]["grantee"] =
            Value::String("urn:example:attacker:wrong-beneficiary".into());
        assert_eq!(
            evaluate_case(case),
            decision("deny", "EXPLICIT_PROHIBITION")
        );
    }

    #[test]
    fn supersession_must_match_requested_purpose_jurisdiction_and_channel() {
        for (field, replacement) in [
            ("purpose", "unapproved-purpose"),
            ("jurisdiction", "US"),
            ("channel", "unapproved-channel"),
        ] {
            let mut document = fixture();
            let case = document["cases"]
                .as_array_mut()
                .expect("cases array")
                .iter_mut()
                .find(|case| case["id"] == "DR-012")
                .expect("supersession vector");
            case[field] = Value::String(replacement.into());
            assert_eq!(
                evaluate_case(case),
                decision("deny", "EXPLICIT_PROHIBITION"),
                "supersession must reject a request with mismatched {field}"
            );
        }
    }

    #[test]
    fn supersession_must_name_a_source_in_the_derivative_parent_manifest() {
        let mut document = fixture();
        let case = document["cases"]
            .as_array_mut()
            .expect("cases array")
            .iter_mut()
            .find(|case| case["id"] == "DR-012")
            .expect("supersession vector");
        case["derivation"]["parents"] = serde_json::json!(["urn:example:unrelated:source"]);
        assert_eq!(
            evaluate_case(case),
            decision("deny", "EXPLICIT_PROHIBITION")
        );
    }

    #[test]
    fn community_label_without_an_operation_grant_does_not_authorize_embedding() {
        let document = fixture();
        let case = document["cases"]
            .as_array()
            .expect("cases array")
            .iter()
            .find(|case| case["id"] == "DR-016")
            .expect("community label vector");
        assert_eq!(
            evaluate_case(case),
            decision("deny", "NO_EXPLICIT_OPERATION_GRANT")
        );
    }

    #[test]
    fn stale_cache_denies_before_a_revoked_grant_can_be_reused() {
        let document = fixture();
        let case = document["cases"]
            .as_array()
            .expect("cases array")
            .iter()
            .find(|case| case["id"] == "DR-008")
            .expect("stale cache vector");
        assert_eq!(
            evaluate_case(case),
            decision("deny", "STALE_POLICY_EPOCH")
        );
    }

    #[test]
    fn indeterminate_is_always_non_permissive() {
        let mut document = fixture();
        document["cases"][0]["source"][0]["policyStatus"] =
            Value::String("unavailable".into());
        let result = evaluate_case(&document["cases"][0]);
        assert_eq!(result, decision("indeterminate", "POLICY_UNAVAILABLE"));
    }
}
