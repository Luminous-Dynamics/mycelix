#![forbid(unsafe_code)]

//! Synthetic reference evaluator for DevSecOps agent authority and protected effects.
//!
//! It separates repository authoring, human approval, merge/release authority,
//! CI workload identity, artifact provenance and production deployment. This
//! fixture evaluator is not a GitHub authorizer, CI PEP or production security
//! certification.

use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};

pub const SUITE_ID: &str = "LISE-DEVSECOPS-AGENT-v0.1";
pub const EVALUATOR_VERSION: &str = "devsecops-agent-reference-v0.1";

fn string<'a>(value: &'a Value, key: &str, context: &str) -> Result<&'a str, String> {
    value
        .get(key)
        .and_then(Value::as_str)
        .filter(|s| !s.trim().is_empty())
        .ok_or_else(|| format!("{context}: '{key}' must be a nonempty string"))
}

fn array<'a>(value: &'a Value, key: &str, context: &str) -> Result<&'a Vec<Value>, String> {
    value
        .get(key)
        .and_then(Value::as_array)
        .ok_or_else(|| format!("{context}: '{key}' must be an array"))
}

fn contains_string(
    value: &Value,
    key: &str,
    expected: &str,
    context: &str,
) -> Result<bool, String> {
    Ok(array(value, key, context)?
        .iter()
        .any(|item| item.as_str() == Some(expected)))
}

fn timestamp<'a>(value: &'a Value, key: &str, context: &str) -> Result<&'a str, String> {
    let result = string(value, key, context)?;
    let b = result.as_bytes();
    if b.len() != 20
        || b[4] != b'-'
        || b[7] != b'-'
        || b[10] != b'T'
        || b[13] != b':'
        || b[16] != b':'
        || b[19] != b'Z'
        || !b
            .iter()
            .enumerate()
            .all(|(i, c)| matches!(i, 4 | 7 | 10 | 13 | 16 | 19) || c.is_ascii_digit())
    {
        return Err(format!("{context}: '{key}' must be YYYY-MM-DDTHH:MM:SSZ"));
    }
    Ok(result)
}

fn decision(decision: &str, reason: &str, violations: Vec<String>) -> Value {
    json!({
        "decision": decision,
        "reason_code": reason,
        "violations": violations
    })
}

fn deny(reason: &str) -> Value {
    decision("DENY", reason, Vec::new())
}

fn hold(reason: &str, violations: Vec<String>) -> Value {
    decision("HOLD", reason, violations)
}

fn permit(reason: &str) -> Value {
    decision("ALLOW_SCOPED", reason, Vec::new())
}

fn check_grant(case: &Value) -> Result<Option<Value>, String> {
    let principal = case.get("principal").ok_or("case: missing principal")?;
    let request = case.get("request").ok_or("case: missing request")?;
    let grant = case.get("grant").ok_or("case: missing grant")?;
    let principal_ref = string(principal, "principal_ref", "principal")?;
    let principal_kind = string(principal, "kind", "principal")?;
    let action = string(request, "action", "request")?;
    let repository = string(request, "repository", "request")?;
    let branch = string(request, "branch", "request")?;
    let environment = string(request, "environment", "request")?;
    let purpose = string(request, "purpose", "request")?;
    let action_at = timestamp(request, "action_at", "request")?;
    let known_at = timestamp(request, "known_at", "request")?;

    if string(grant, "principal_ref", "grant")? != principal_ref {
        return Ok(Some(deny("grant.principal_mismatch")));
    }
    let state = string(grant, "state", "grant")?;
    match state {
        "revoked" => return Ok(Some(deny("grant.revoked"))),
        "pending" | "unknown" => return Ok(Some(hold("grant.status_unknown", Vec::new()))),
        "active" => {}
        _ => return Ok(Some(hold("grant.status_unrecognized", Vec::new()))),
    }
    let observed_at = timestamp(grant, "observed_at", "grant")?;
    if observed_at > known_at {
        return Ok(Some(hold("grant.not_yet_observed", Vec::new())));
    }
    let valid_from = timestamp(grant, "valid_from", "grant")?;
    let valid_through = timestamp(grant, "valid_through", "grant")?;
    if action_at < valid_from {
        return Ok(Some(deny("grant.not_yet_valid")));
    }
    if action_at > valid_through {
        return Ok(Some(deny("grant.expired")));
    }
    if !contains_string(grant, "actions", action, "grant")?
        || !contains_string(grant, "repositories", repository, "grant")?
        || !contains_string(grant, "branches", branch, "grant")?
        || !contains_string(grant, "environments", environment, "grant")?
        || !contains_string(grant, "purposes", purpose, "grant")?
    {
        return Ok(Some(deny("grant.scope_mismatch")));
    }
    let policy = case.get("policy").ok_or("case: missing policy")?;
    if string(grant, "policy_digest", "grant")? != string(policy, "current_digest", "policy")? {
        return Ok(Some(deny("policy.digest_drift")));
    }
    let _ = principal_kind;
    Ok(None)
}

fn check_human_approval(case: &Value) -> Result<Option<Value>, String> {
    let principal = case.get("principal").ok_or("case: missing principal")?;
    let request = case.get("request").ok_or("case: missing request")?;
    let Some(approval) = case.get("human_approval").filter(|v| v.is_object()) else {
        return Ok(Some(deny("human_approval.missing")));
    };
    let principal_ref = string(principal, "principal_ref", "principal")?;
    let principal_kind = string(principal, "kind", "principal")?;
    if principal_kind == "human" {
        return Ok(Some(deny("human_approval.actor_not_agent_workload")));
    }
    if string(approval, "state", "human approval")? != "approved" {
        return Ok(Some(deny("human_approval.not_approved")));
    }
    if string(approval, "approver_ref", "human approval")? == principal_ref {
        return Ok(Some(deny("human_approval.self_approval")));
    }
    let action_at = timestamp(request, "action_at", "request")?;
    let known_at = timestamp(request, "known_at", "request")?;
    let valid_through = timestamp(approval, "valid_through", "human approval")?;
    if action_at > valid_through {
        return Ok(Some(deny("human_approval.expired")));
    }
    let status_events = array(approval, "status_events", "human approval")?;
    let mut visible_statuses = Vec::new();
    for event in status_events {
        let effective_at = timestamp(event, "effective_at", "approval status event")?;
        let observed_at = timestamp(event, "observed_at", "approval status event")?;
        if effective_at <= action_at {
            if observed_at > known_at {
                return Ok(Some(hold(
                    "human_approval.revocation_not_yet_observed",
                    Vec::new(),
                )));
            }
            visible_statuses.push((
                effective_at,
                observed_at,
                string(event, "status", "approval status event")?,
            ));
        }
    }
    visible_statuses.sort_by(|a, b| (a.0, a.1).cmp(&(b.0, b.1)));
    if let Some((_, _, status)) = visible_statuses.last()
        && *status == "revoked"
    {
        return Ok(Some(deny("human_approval.revoked")));
    }
    if string(approval, "approver_kind", "human approval")? != "human"
        || approval.get("reviewer_qualified").and_then(Value::as_bool) != Some(true)
    {
        return Ok(Some(deny("human_approval.reviewer_not_qualified")));
    }
    let approved_at = timestamp(approval, "approved_at", "human approval")?;
    let observed_at = timestamp(approval, "observed_at", "human approval")?;
    if approved_at > action_at || observed_at > known_at {
        return Ok(Some(hold(
            "human_approval.not_valid_at_decision_time",
            Vec::new(),
        )));
    }
    if string(approval, "approved_head_sha", "human approval")?
        != string(request, "current_head_sha", "request")?
    {
        return Ok(Some(deny("human_approval.head_sha_mismatch")));
    }
    let policy = case.get("policy").ok_or("case: missing policy")?;
    if string(approval, "policy_digest", "human approval")?
        != string(policy, "current_digest", "policy")?
    {
        return Ok(Some(deny("human_approval.policy_digest_mismatch")));
    }
    if string(approval, "repository", "human approval")?
        != string(request, "repository", "request")?
        || string(approval, "branch", "human approval")? != string(request, "branch", "request")?
        || string(approval, "environment", "human approval")?
            != string(request, "environment", "request")?
        || string(approval, "action", "human approval")? != string(request, "action", "request")?
    {
        return Ok(Some(deny("human_approval.operation_scope_mismatch")));
    }
    Ok(None)
}

fn check_workload_identity(case: &Value) -> Result<Option<Value>, String> {
    let principal = case.get("principal").ok_or("case: missing principal")?;
    let request = case.get("request").ok_or("case: missing request")?;
    let Some(identity) = case.get("workload_identity").filter(|v| v.is_object()) else {
        return Ok(Some(deny("workload_identity.missing")));
    };
    if string(principal, "kind", "principal")? != "ci_workload" {
        return Ok(Some(deny(
            "workload_identity.direct_agent_deploy_forbidden",
        )));
    }
    let checks = [
        ("subject_ref", "principal_ref", principal),
        ("audience", "expected_audience", request),
        ("repository", "repository", request),
        ("workflow_ref", "workflow_ref", request),
        ("workflow_sha", "workflow_sha", request),
        ("sub", "expected_subject_claim", request),
        ("repository_id", "expected_repository_id", request),
        ("repository_owner_id", "expected_owner_id", request),
        ("job_workflow_ref", "expected_job_workflow_ref", request),
        ("run_id", "workflow_run_id", request),
    ];
    for (identity_key, expected_key, expected_source) in checks {
        if string(identity, identity_key, "workload identity")?
            != string(expected_source, expected_key, "workload identity binding")?
        {
            return Ok(Some(deny("workload_identity.binding_mismatch")));
        }
    }
    let action_at = timestamp(request, "action_at", "request")?;
    if identity.get("run_attempt").and_then(Value::as_u64)
        != request.get("expected_run_attempt").and_then(Value::as_u64)
    {
        return Ok(Some(deny("workload_identity.run_attempt_mismatch")));
    }
    if string(identity, "environment", "workload identity")?
        != string(request, "environment", "request")?
    {
        return Ok(Some(deny("workload_identity.environment_mismatch")));
    }
    let known_at = timestamp(request, "known_at", "request")?;
    if timestamp(identity, "observed_at", "workload identity")? > known_at {
        return Ok(Some(hold("workload_identity.not_yet_observed", Vec::new())));
    }
    if timestamp(identity, "valid_from", "workload identity")? > action_at {
        return Ok(Some(deny("workload_identity.not_yet_valid")));
    }
    if timestamp(identity, "valid_through", "workload identity")? < action_at {
        return Ok(Some(deny("workload_identity.expired")));
    }
    if !contains_string(
        case.get("policy").ok_or("case: missing policy")?,
        "trusted_oidc_issuers",
        string(identity, "issuer", "workload identity")?,
        "policy",
    )? {
        return Ok(Some(deny("workload_identity.untrusted_issuer")));
    }
    if string(identity, "state", "workload identity")? != "verified" {
        return Ok(Some(deny("workload_identity.not_verified")));
    }
    Ok(None)
}

fn check_artifact_attestation(case: &Value) -> Result<Option<Value>, String> {
    let request = case.get("request").ok_or("case: missing request")?;
    let Some(attestation) = case.get("artifact_attestation").filter(|v| v.is_object()) else {
        return Ok(Some(deny("artifact_attestation.missing")));
    };
    if string(attestation, "state", "artifact attestation")? != "verified" {
        return Ok(Some(deny("artifact_attestation.not_verified")));
    }

    let policy = case.get("policy").ok_or("case: missing policy")?;
    let signer_id = string(attestation, "signer_id", "artifact attestation")?;
    let builder_id = string(attestation, "builder_id", "artifact attestation")?;
    let trusted_pairs = array(policy, "trusted_signer_builder_pairs", "policy")?;
    let pair_trusted = trusted_pairs.iter().any(|pair| {
        pair.get("signer_id").and_then(Value::as_str) == Some(signer_id)
            && pair.get("builder_id").and_then(Value::as_str) == Some(builder_id)
    });
    if !pair_trusted {
        return Ok(Some(deny("artifact_attestation.untrusted_signer_builder_pair")));
    }

    if string(attestation, "subject_digest", "artifact attestation")?
        != string(request, "artifact_digest", "request")?
        || string(attestation, "commit_sha", "artifact attestation")?
            != string(request, "current_head_sha", "request")?
        || string(attestation, "predicate_type", "artifact attestation")?
            != "https://slsa.dev/provenance/v1"
        || string(attestation, "workflow_ref", "artifact attestation")?
            != string(request, "workflow_ref", "request")?
        || string(attestation, "workflow_sha", "artifact attestation")?
            != string(request, "workflow_sha", "request")?
        || string(attestation, "invocation_id", "artifact attestation")?
            != string(request, "workflow_run_id", "request")?
    {
        return Ok(Some(deny(
            "artifact_attestation.subject_builder_or_invocation_mismatch",
        )));
    }

    let required_dependencies = array(request, "required_dependencies", "request")?;
    let resolved_dependencies = array(
        attestation,
        "resolved_dependencies",
        "artifact attestation",
    )?;
    for required in required_dependencies {
        let uri = string(required, "uri", "required dependency")?;
        let digest = required
            .get("digest")
            .ok_or("required dependency: missing digest")?;
        let resolved = resolved_dependencies.iter().any(|candidate| {
            candidate.get("uri").and_then(Value::as_str) == Some(uri)
                && candidate.get("digest") == Some(digest)
        });
        if !resolved {
            return Ok(Some(deny(
                "artifact_attestation.dependency_missing_or_mismatched",
            )));
        }
    }

    let known_at = timestamp(request, "known_at", "request")?;
    if timestamp(attestation, "observed_at", "artifact attestation")? > known_at {
        return Ok(Some(hold(
            "artifact_attestation.not_yet_observed",
            Vec::new(),
        )));
    }
    Ok(None)
}

fn evaluate_case(case: &Value) -> Result<Value, String> {
    let request = case.get("request").ok_or("case: missing request")?;
    if string(request, "prior_effect_state", "request")? == "unknown_after_dispatch" {
        let retry_count = request
            .get("retry_count")
            .and_then(Value::as_u64)
            .unwrap_or(u64::MAX);
        let mut violations = Vec::new();
        if retry_count > 0 {
            violations.push("blind_retry_after_unknown".to_owned());
        }
        return Ok(hold(
            "operation.outcome_unknown_requires_reconciliation",
            violations,
        ));
    }

    if let Some(result) = check_grant(case)? {
        return Ok(result);
    }
    let action = string(request, "action", "request")?;
    if action == "approve_pull_request" {
        let principal = case.get("principal").ok_or("case: missing principal")?;
        if string(principal, "kind", "principal")? != "human" {
            return Ok(deny("human_approval.agent_cannot_approve"));
        }
        if principal.get("reviewer_qualified").and_then(Value::as_bool) != Some(true) {
            return Ok(deny("human_approval.reviewer_not_qualified"));
        }
        if string(request, "reviewed_head_sha", "request")?
            != string(request, "current_head_sha", "request")?
        {
            return Ok(deny("human_approval.head_sha_mismatch"));
        }
        return Ok(permit("qualified_human_approval_bound_to_current_head"));
    }

    let protected_actions = ["merge_pull_request", "create_release", "deploy_production"];
    if protected_actions.contains(&action)
        && let Some(result) = check_human_approval(case)?
    {
        return Ok(result);
    }
    if action == "deploy_production" {
        if let Some(result) = check_workload_identity(case)? {
            return Ok(result);
        }
        if let Some(result) = check_artifact_attestation(case)? {
            return Ok(result);
        }
    }
    Ok(permit("exact_scope_and_prerequisites_satisfied"))
}

fn case_index(cases: &[Value]) -> Result<BTreeMap<String, &Value>, String> {
    let mut result = BTreeMap::new();
    for case in cases {
        let id = string(case, "case_id", "scenario")?.to_owned();
        if result.insert(id.clone(), case).is_some() {
            return Err(format!("duplicate scenario id '{id}'"));
        }
    }
    Ok(result)
}

/// Evaluates all candidate-visible scenarios against a separate known-answer oracle.
pub fn evaluate_bundle(cases: &Value, oracle: &Value) -> Value {
    let result = (|| -> Result<Value, String> {
        let raw_cases = array(cases, "cases", "case bundle")?;
        let raw_oracle = array(oracle, "expected", "oracle")?;
        let case_map = case_index(raw_cases)?;
        let oracle_map = case_index(raw_oracle)?;
        let case_ids: BTreeSet<String> = case_map.keys().cloned().collect();
        let oracle_ids: BTreeSet<String> = oracle_map.keys().cloned().collect();
        if case_ids != oracle_ids {
            return Err("case/oracle scenario ID sets differ".to_owned());
        }
        let mut reports = Vec::new();
        let mut matched = 0usize;
        let mut violations = 0usize;
        for id in case_ids {
            let observed = evaluate_case(case_map[&id])?;
            let expected = oracle_map[&id];
            let conformant = observed.get("decision") == expected.get("expected_decision")
                && observed.get("reason_code") == expected.get("expected_reason_code")
                && observed.get("violations") == expected.get("expected_violations");
            if conformant {
                matched += 1;
            }
            let found_violations = observed
                .get("violations")
                .and_then(Value::as_array)
                .map(Vec::len)
                .unwrap_or(0);
            violations += found_violations;
            reports.push(json!({
                "case_id": id,
                "expected_decision": expected["expected_decision"],
                "observed": observed,
                "conformant": conformant
            }));
        }
        Ok(json!({
            "suite_id": SUITE_ID,
            "evaluator_version": EVALUATOR_VERSION,
            "case_count": reports.len(),
            "conformant_case_count": matched,
            "violation_count": violations,
            "cases": reports,
            "conformant": matched == reports.len(),
            "dispatch_eligible": false,
            "claim_ceiling": "Synthetic DevSecOps reference semantics only; no repository, CI, signer, workload identity or deployment authority is authenticated."
        }))
    })();
    result.unwrap_or_else(|error| {
        json!({
            "suite_id": SUITE_ID,
            "evaluator_version": EVALUATOR_VERSION,
            "status": "INVALID_INPUT",
            "conformant": false,
            "dispatch_eligible": false,
            "errors": [error],
            "claim_ceiling": "No benchmark result from invalid inputs."
        })
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bundle() -> (Value, Value) {
        let cases: Value =
            serde_json::from_str(include_str!("../fixtures/cases.json")).expect("cases JSON");
        let oracle: Value =
            serde_json::from_str(include_str!("../fixtures/oracle.json")).expect("oracle JSON");
        (cases, oracle)
    }

    #[test]
    fn frozen_reference_bundle_matches_every_known_answer() {
        let (cases, oracle) = bundle();
        let report = evaluate_bundle(&cases, &oracle);
        assert_eq!(report["conformant"], true);
        assert_eq!(report["case_count"], 19);
        assert_eq!(report["conformant_case_count"], 19);
    }

    #[test]
    fn immutable_subject_rejects_repository_namespace_recycling() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-017")
            .expect("case");
        assert_eq!(
            evaluate_case(case).expect("decision")["reason_code"],
            "workload_identity.binding_mismatch"
        );
    }

    #[test]
    fn reusable_workflow_identity_is_bound_to_the_approved_workflow() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-018")
            .expect("case");
        assert_eq!(
            evaluate_case(case).expect("decision")["reason_code"],
            "workload_identity.binding_mismatch"
        );
    }

    #[test]
    fn workflow_run_attempt_is_bound_to_the_expected_invocation() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-019")
            .expect("case");
        assert_eq!(
            evaluate_case(case).expect("decision")["reason_code"],
            "workload_identity.run_attempt_mismatch"
        );
    }

    #[test]
    fn expired_ci_workload_identity_cannot_authorize_deployment() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-016")
            .expect("case");
        assert_eq!(
            evaluate_case(case).expect("decision")["reason_code"],
            "workload_identity.expired"
        );
    }

    #[test]
    fn qualified_human_reviewer_can_approve_exact_current_head() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-013")
            .expect("case");
        let result = evaluate_case(case).expect("decision");
        assert_eq!(result["decision"], "ALLOW_SCOPED");
        assert_eq!(
            result["reason_code"],
            "qualified_human_approval_bound_to_current_head"
        );
    }

    #[test]
    fn expired_and_revoked_human_approvals_cannot_authorize_merge() {
        let (cases, _) = bundle();
        let expired = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-014")
            .expect("case");
        assert_eq!(
            evaluate_case(expired).expect("decision")["reason_code"],
            "human_approval.expired"
        );
        let revoked = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-015")
            .expect("case");
        assert_eq!(
            evaluate_case(revoked).expect("decision")["reason_code"],
            "human_approval.revoked"
        );
    }

    #[test]
    fn agent_can_author_a_feature_branch_without_merge_authority() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-001")
            .expect("case");
        let result = evaluate_case(case).expect("decision");
        assert_eq!(result["decision"], "ALLOW_SCOPED");
    }

    #[test]
    fn agent_cannot_approve_its_own_pull_request() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-003")
            .expect("case");
        assert_eq!(
            evaluate_case(case).expect("decision")["reason_code"],
            "human_approval.self_approval"
        );
    }

    #[test]
    fn approval_must_bind_the_current_head_and_policy() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-004")
            .expect("case");
        assert_eq!(
            evaluate_case(case).expect("decision")["reason_code"],
            "human_approval.head_sha_mismatch"
        );
    }

    #[test]
    fn production_deploy_requires_ci_workload_audience_and_subject() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-005")
            .expect("case");
        assert_eq!(
            evaluate_case(case).expect("decision")["reason_code"],
            "workload_identity.binding_mismatch"
        );
    }

    #[test]
    fn valid_production_deploy_requires_human_approval_and_attestation() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-006")
            .expect("case");
        assert_eq!(
            evaluate_case(case).expect("decision")["decision"],
            "ALLOW_SCOPED"
        );
    }

    #[test]
    fn artifact_attestation_requires_an_exact_trusted_signer_builder_pair() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-020")
            .expect("case");
        assert_eq!(
            evaluate_case(case).expect("decision")["reason_code"],
            "artifact_attestation.untrusted_signer_builder_pair"
        );
    }

    #[test]
    fn artifact_attestation_requires_exact_resolved_dependency_digests() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-021")
            .expect("case");
        assert_eq!(
            evaluate_case(case).expect("decision")["reason_code"],
            "artifact_attestation.dependency_missing_or_mismatched"
        );
    }

    #[test]
    fn artifact_digest_mismatch_blocks_production_deploy() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-007")
            .expect("case");
        assert_eq!(
            evaluate_case(case).expect("decision")["reason_code"],
            "artifact_attestation.subject_or_builder_mismatch"
        );
    }

    #[test]
    fn unknown_deployment_outcome_is_held_and_never_retried() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-008")
            .expect("case");
        let result = evaluate_case(case).expect("decision");
        assert_eq!(result["decision"], "HOLD");
        assert_eq!(result["violations"], json!([]));
    }

    #[test]
    fn retry_after_unknown_deployment_is_detected() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-009")
            .expect("case");
        let result = evaluate_case(case).expect("decision");
        assert_eq!(result["decision"], "HOLD");
        assert_eq!(result["violations"], json!(["blind_retry_after_unknown"]));
    }

    #[test]
    fn prompt_injection_does_not_expand_an_agent_grant() {
        let (cases, _) = bundle();
        let case = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-010")
            .expect("case");
        assert_eq!(
            evaluate_case(case).expect("decision")["reason_code"],
            "grant.scope_mismatch"
        );
    }

    #[test]
    fn revocation_and_policy_drift_fail_closed() {
        let (cases, _) = bundle();
        let revoked = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-011")
            .expect("case");
        assert_eq!(
            evaluate_case(revoked).expect("decision")["reason_code"],
            "grant.revoked"
        );
        let drifted = cases["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|c| c["case_id"] == "DOPS-012")
            .expect("case");
        assert_eq!(
            evaluate_case(drifted).expect("decision")["reason_code"],
            "policy.digest_drift"
        );
    }

    #[test]
    fn candidate_cannot_omit_or_duplicate_scenarios() {
        let (mut cases, oracle) = bundle();
        cases["cases"].as_array_mut().expect("cases").pop();
        assert_eq!(evaluate_bundle(&cases, &oracle)["status"], "INVALID_INPUT");

        let (mut cases, oracle) = bundle();
        let duplicate = cases["cases"][0].clone();
        cases["cases"]
            .as_array_mut()
            .expect("cases")
            .push(duplicate);
        assert_eq!(evaluate_bundle(&cases, &oracle)["status"], "INVALID_INPUT");
    }
}
