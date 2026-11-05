#![forbid(unsafe_code)]

use jsonschema::draft202012;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

const SUBJECT_DOMAIN: &[u8] = b"mycelix:fin-acc-review:subject:v1\0";
const POLICY_DOMAIN: &[u8] = b"mycelix:fin-acc-review:policy:v1\0";

pub fn verify(input_bytes: &[u8], schema_bytes: &[u8]) -> Value {
    let input: Value = match serde_json::from_slice(input_bytes) {
        Ok(value) => value,
        Err(error) => {
            return report_error("invalid_input", format!("candidate JSON: {error}"));
        }
    };
    let schema: Value = match serde_json::from_slice(schema_bytes) {
        Ok(value) => value,
        Err(error) => {
            return report_error("invalid_schema", format!("schema JSON: {error}"));
        }
    };
    if let Err(error) = jsonschema::draft202012::meta::validate(&schema) {
        return report_error("invalid_schema", format!("Draft 2020-12 schema: {error}"));
    }
    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
    {
        Ok(value) => value,
        Err(error) => {
            return report_error("invalid_schema", format!("schema compilation: {error}"));
        }
    };
    let validation_errors = validator
        .iter_errors(&input)
        .map(|error| format!("{error} at {}", error.instance_path()))
        .collect::<Vec<_>>();
    if !validation_errors.is_empty() {
        return json!({
            "report_schema": "mycelix.fin.accounting-review.report.v1",
            "disposition": "invalid_input",
            "subject_digest": Value::Null,
            "review_policy_digest": Value::Null,
            "role_counts": [],
            "missing_roles": [],
            "blocking_reasons": validation_errors,
            "limitations": claim_limitations(),
            "external_authority_verified": false,
            "human_review_sufficiency_verified": false,
            "can_approve_close": false
        });
    }
    let report = verify_value(&input);
    validate_report_schema(&report)
}

fn validate_report_schema(report: &Value) -> Value {
    let schema: Value = match serde_json::from_str(include_str!("../report.schema.json")) {
        Ok(schema) => schema,
        Err(error) => {
            return report_error("invalid_report", format!("embedded report schema: {error}"));
        }
    };
    if let Err(error) = jsonschema::draft202012::meta::validate(&schema) {
        return report_error(
            "invalid_report",
            format!("embedded report schema meta-validation: {error}"),
        );
    }
    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
    {
        Ok(validator) => validator,
        Err(error) => {
            return report_error(
                "invalid_report",
                format!("embedded report schema compilation: {error}"),
            );
        }
    };
    let errors = validator
        .iter_errors(report)
        .map(|error| format!("{error} at {}", error.instance_path()))
        .collect::<Vec<_>>();
    if errors.is_empty() {
        report.clone()
    } else {
        report_error("invalid_report", errors.join("; "))
    }
}

pub fn verify_value(input: &Value) -> Value {
    let subject = &input["subject"];
    let policy = &input["review_policy"];
    let computed_subject_digest = digest_value(SUBJECT_DOMAIN, subject);
    let computed_policy_digest = digest_value(POLICY_DOMAIN, policy);
    let mut blockers = BTreeSet::<String>::new();
    let mut counts = BTreeMap::<String, BTreeSet<String>>::new();
    let mut accepted_events = BTreeSet::<String>::new();
    let mut seen_event_ids = BTreeSet::<String>::new();
    let mut limited_approvals = BTreeSet::<String>::new();

    let prepared_by = subject["prepared_by"]["principal_ref"]
        .as_str()
        .unwrap_or_default();
    let policy_owner = policy["policy_owner_principal_ref"]
        .as_str()
        .unwrap_or_default();
    counts
        .entry("preparer".to_owned())
        .or_default()
        .insert(prepared_by.to_owned());
    counts
        .entry("policy_owner".to_owned())
        .or_default()
        .insert(policy_owner.to_owned());

    if subject["review_policy_ref"] != policy["policy_ref"] {
        blockers.insert("review_policy_reference_mismatch".to_owned());
    }
    let subject_policy_digest = subject["review_policy_digest"].as_str().unwrap_or_default();
    if subject_policy_digest != computed_policy_digest {
        blockers.insert("review_policy_digest_mismatch".to_owned());
    }

    let subject_entity = subject["reporting_entity_ref"].as_str().unwrap_or_default();
    let subject_period = subject["reporting_period"].as_str().unwrap_or_default();
    let subject_procedure = subject["procedure_ref"].as_str().unwrap_or_default();
    let subject_authority_frontier = subject["authority_frontier_digest"]
        .as_str()
        .unwrap_or_default();
    let subject_pairs = sorted_strings(&subject["assertion_pair_ids"]);

    for event in input["review_events"]
        .as_array()
        .cloned()
        .unwrap_or_default()
    {
        let event_id = event["event_id"].as_str().unwrap_or_default().to_owned();
        if !seen_event_ids.insert(event_id.clone()) {
            blockers.insert(format!("duplicate_event_id:{event_id}"));
            continue;
        }
        let principal = event["principal_ref"]
            .as_str()
            .unwrap_or_default()
            .to_owned();
        let role = event["role"].as_str().unwrap_or_default().to_owned();
        let decision = event["decision"].as_str().unwrap_or_default();

        if event["subject_digest"].as_str() != Some(computed_subject_digest.as_str()) {
            blockers.insert(format!("stale_subject_binding:{event_id}"));
            continue;
        }
        if event["policy_digest"].as_str() != Some(computed_policy_digest.as_str()) {
            blockers.insert(format!("stale_policy_binding:{event_id}"));
            continue;
        }
        if event["authority_frontier_digest"].as_str() != Some(subject_authority_frontier) {
            blockers.insert(format!("stale_authority_frontier:{event_id}"));
            continue;
        }
        if event["reporting_entity_ref"].as_str() != Some(subject_entity)
            || event["reporting_period"].as_str() != Some(subject_period)
            || event["procedure_ref"].as_str() != Some(subject_procedure)
            || sorted_strings(&event["assertion_pair_ids"]) != subject_pairs
        {
            blockers.insert(format!("review_scope_mismatch:{event_id}"));
            continue;
        }
        let recorded_at = event["recorded_at_unix_ms"].as_u64().unwrap_or(0);
        let prepared_at = subject["prepared_at_unix_ms"].as_u64().unwrap_or(u64::MAX);
        if recorded_at < prepared_at {
            blockers.insert(format!("review_predates_workpaper:{event_id}"));
            continue;
        }

        if decision == "rejected" || decision == "request_changes" {
            blockers.insert(format!("adverse_review_decision:{role}:{principal}"));
            continue;
        }

        let compatible = matches!(
            (role.as_str(), decision),
            ("performer", "performed")
                | ("reviewer", "reviewed")
                | ("approver", "approved")
                | ("approver", "approved_with_limitation")
        );
        if !compatible {
            blockers.insert(format!(
                "role_decision_mismatch:{role}:{decision}:{principal}"
            ));
            continue;
        }
        if decision == "approved_with_limitation" {
            limited_approvals.insert(principal.clone());
        }
        counts.entry(role).or_default().insert(principal);
        accepted_events.insert(event_id);
    }

    for constraint in policy["separation_constraints"]
        .as_array()
        .cloned()
        .unwrap_or_default()
    {
        let left = constraint["left_role"].as_str().unwrap_or_default();
        let right = constraint["right_role"].as_str().unwrap_or_default();
        let left_principals = counts.get(left).cloned().unwrap_or_default();
        let right_principals = counts.get(right).cloned().unwrap_or_default();
        for principal in left_principals.intersection(&right_principals) {
            blockers.insert(format!(
                "role_separation_violation:{left}:{right}:{principal}"
            ));
        }
    }

    let mut missing_roles = Vec::<Value>::new();
    for requirement in policy["required_role_counts"]
        .as_array()
        .cloned()
        .unwrap_or_default()
    {
        let role = requirement["role"].as_str().unwrap_or_default();
        let minimum = requirement["minimum_unique_principals"]
            .as_u64()
            .unwrap_or(1) as usize;
        let actual = counts.get(role).map(BTreeSet::len).unwrap_or(0);
        if actual < minimum {
            missing_roles.push(json!({
                "role": role,
                "required_unique_principals": minimum,
                "observed_unique_principals": actual
            }));
        }
    }

    let stale = blockers.iter().any(|reason| reason.starts_with("stale_"));
    let policy_mismatch = blockers.contains("review_policy_reference_mismatch")
        || blockers.contains("review_policy_digest_mismatch");
    let scope_mismatch = blockers
        .iter()
        .any(|reason| reason.starts_with("review_scope_mismatch"));
    let separation_violation = blockers
        .iter()
        .any(|reason| reason.starts_with("role_separation_violation"));
    let adverse = blockers
        .iter()
        .any(|reason| reason.starts_with("adverse_review_decision"));
    let role_mismatch = blockers
        .iter()
        .any(|reason| reason.starts_with("role_decision_mismatch"));
    let event_integrity_failure = blockers.iter().any(|reason| {
        reason.starts_with("duplicate_event_id") || reason.starts_with("review_predates_workpaper")
    });

    let disposition = if stale || policy_mismatch {
        "stale_subject_or_policy"
    } else if scope_mismatch {
        "review_scope_mismatch"
    } else if separation_violation {
        "blocked_by_role_separation"
    } else if adverse {
        "blocked_by_adverse_review"
    } else if role_mismatch || event_integrity_failure {
        "blocked_by_review_contract"
    } else if !missing_roles.is_empty() {
        "awaiting_required_roles"
    } else {
        "structurally_ready_pending_external_authority"
    };

    json!({
        "report_schema": "mycelix.fin.accounting-review.report.v1",
        "disposition": disposition,
        "subject_digest": computed_subject_digest,
        "review_policy_digest": computed_policy_digest,
        "role_counts": counts.iter().map(|(role, principals)| json!({
            "role": role,
            "unique_principal_count": principals.len(),
            "principal_refs": principals.iter().collect::<Vec<_>>()
        })).collect::<Vec<_>>(),
        "accepted_event_ids": accepted_events.iter().collect::<Vec<_>>(),
        "limited_approval_principal_refs": limited_approvals.iter().collect::<Vec<_>>(),
        "missing_roles": missing_roles,
        "blocking_reasons": blockers.into_iter().collect::<Vec<_>>(),
        "limitations": claim_limitations(),
        "external_authority_verified": false,
        "human_review_sufficiency_verified": false,
        "can_approve_close": false
    })
}

fn sorted_strings(value: &Value) -> Vec<String> {
    let mut values = value
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .map(str::to_owned)
        .collect::<Vec<_>>();
    values.sort();
    values
}

fn digest_value(domain: &[u8], value: &Value) -> String {
    let mut bytes = domain.to_vec();
    write_canonical_json(value, &mut bytes);
    hex::encode(Sha256::digest(bytes))
}

fn write_canonical_json(value: &Value, out: &mut Vec<u8>) {
    match value {
        Value::Null => out.extend_from_slice(b"null"),
        Value::Bool(value) => {
            if *value {
                out.extend_from_slice(b"true");
            } else {
                out.extend_from_slice(b"false");
            }
        }
        Value::Number(value) => out.extend_from_slice(value.to_string().as_bytes()),
        Value::String(value) => out.extend_from_slice(
            serde_json::to_string(value)
                .expect("JSON string serialization")
                .as_bytes(),
        ),
        Value::Array(values) => {
            out.push(b'[');
            for (index, value) in values.iter().enumerate() {
                if index > 0 {
                    out.push(b',');
                }
                write_canonical_json(value, out);
            }
            out.push(b']');
        }
        Value::Object(values) => {
            let sorted = values.iter().collect::<BTreeMap<_, _>>();
            out.push(b'{');
            for (index, (key, value)) in sorted.iter().enumerate() {
                if index > 0 {
                    out.push(b',');
                }
                out.extend_from_slice(
                    serde_json::to_string(key)
                        .expect("JSON key serialization")
                        .as_bytes(),
                );
                out.push(b':');
                write_canonical_json(value, out);
            }
            out.push(b'}');
        }
    }
}

fn claim_limitations() -> Vec<&'static str> {
    vec![
        "principal_ref equality is structural only; identity alias resolution is external",
        "identity_evidence_ref and authority_evidence_ref are opaque references and are not authenticated or checked for current authority",
        "delegation, revocation and approval withdrawal are not interpreted by v1",
        "a reviewed or approved decision does not prove human comprehension or professional sufficiency",
        "this verifier never grants journal posting, close release, tax filing or external execution authority",
    ]
}

fn report_error(disposition: &str, error: String) -> Value {
    json!({
        "report_schema": "mycelix.fin.accounting-review.report.v1",
        "disposition": disposition,
        "subject_digest": Value::Null,
        "review_policy_digest": Value::Null,
        "role_counts": [],
        "accepted_event_ids": [],
        "limited_approval_principal_refs": [],
        "missing_roles": [],
        "blocking_reasons": [error],
        "limitations": claim_limitations(),
        "external_authority_verified": false,
        "human_review_sufficiency_verified": false,
        "can_approve_close": false
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sha(value: &Value, domain: &[u8]) -> String {
        digest_value(domain, value)
    }

    fn base_record() -> Value {
        let policy = json!({
            "policy_ref": "review-policy:synthetic-close-v1",
            "policy_version": "1",
            "policy_owner_principal_ref": "principal:policy-owner",
            "required_role_counts": [
                {"role": "reviewer", "minimum_unique_principals": 1},
                {"role": "approver", "minimum_unique_principals": 1}
            ],
            "separation_constraints": [
                {"left_role": "preparer", "right_role": "reviewer"},
                {"left_role": "preparer", "right_role": "approver"},
                {"left_role": "reviewer", "right_role": "approver"},
                {"left_role": "policy_owner", "right_role": "approver"}
            ]
        });
        let policy_digest = sha(&policy, POLICY_DOMAIN);
        let subject = json!({
            "subject_id": "workpaper:synthetic:rev-1",
            "workpaper_ref": "workpaper:synthetic:rev-1",
            "workpaper_digest": "a".repeat(64),
            "reporting_entity_ref": "entity:synthetic-za-001",
            "reporting_period": "2026-10",
            "book_domain": "financial",
            "assertion_pair_ids": ["risk:ap:assertion:completeness", "risk:ap:assertion:existence"],
            "procedure_ref": "procedure:ap-reconciliation:v1",
            "source_frontier_digest": "b".repeat(64),
            "evidence_frontier_digest": "c".repeat(64),
            "parent_workpaper_digests": ["d".repeat(64)],
            "accounting_profile_digest": "e".repeat(64),
            "accounting_policy_digest": "f".repeat(64),
            "review_policy_ref": "review-policy:synthetic-close-v1",
            "review_policy_digest": policy_digest,
            "knowledge_frontier_digest": "1".repeat(64),
            "authority_frontier_digest": "2".repeat(64),
            "prepared_by": {"principal_ref": "principal:preparer", "actor_kind": "human"},
            "prepared_at_unix_ms": 1000
        });
        let subject_digest = sha(&subject, SUBJECT_DOMAIN);
        let common = json!({
            "subject_digest": subject_digest,
            "policy_digest": policy_digest,
            "reporting_entity_ref": "entity:synthetic-za-001",
            "reporting_period": "2026-10",
            "assertion_pair_ids": ["risk:ap:assertion:completeness", "risk:ap:assertion:existence"],
            "procedure_ref": "procedure:ap-reconciliation:v1",
            "authority_frontier_digest": "2".repeat(64),
            "identity_evidence_ref": "identity-evidence:synthetic",
            "authority_evidence_ref": "authority-evidence:synthetic",
            "recorded_at_unix_ms": 2000
        });
        let mut reviewer = common.clone();
        reviewer["event_id"] = json!("review-event:reviewer-1");
        reviewer["role"] = json!("reviewer");
        reviewer["principal_ref"] = json!("principal:reviewer");
        reviewer["decision"] = json!("reviewed");
        let mut approver = common;
        approver["event_id"] = json!("review-event:approver-1");
        approver["role"] = json!("approver");
        approver["principal_ref"] = json!("principal:approver");
        approver["decision"] = json!("approved");
        json!({
            "schema_id": "mycelix.fin.accounting-review",
            "schema_version": "1.0.0",
            "subject": subject,
            "review_policy": policy,
            "review_events": [reviewer, approver]
        })
    }

    fn disposition(record: &Value) -> String {
        verify_value(record)["disposition"]
            .as_str()
            .unwrap()
            .to_owned()
    }

    #[test]
    fn exact_subject_and_disjoint_roles_are_structurally_ready_but_never_approved() {
        let report = verify_value(&base_record());
        assert_eq!(
            report["disposition"],
            "structurally_ready_pending_external_authority"
        );
        assert_eq!(report["external_authority_verified"], false);
        assert_eq!(report["human_review_sufficiency_verified"], false);
        assert_eq!(report["can_approve_close"], false);
    }

    #[test]
    fn preparer_cannot_review_when_policy_requires_separation() {
        let mut record = base_record();
        record["review_events"][0]["principal_ref"] = json!("principal:preparer");
        assert_eq!(disposition(&record), "blocked_by_role_separation");
    }

    #[test]
    fn reviewer_and_approver_must_be_distinct_when_policy_requires_it() {
        let mut record = base_record();
        record["review_events"][1]["principal_ref"] = json!("principal:reviewer");
        assert_eq!(disposition(&record), "blocked_by_role_separation");
    }

    #[test]
    fn one_canonical_principal_counts_once_toward_multi_reviewer_quorum() {
        let mut record = base_record();
        record["review_policy"]["required_role_counts"][0]["minimum_unique_principals"] = json!(2);
        let mut duplicate = record["review_events"][0].clone();
        duplicate["event_id"] = json!("review-event:reviewer-duplicate-presentation");
        record["review_events"]
            .as_array_mut()
            .unwrap()
            .push(duplicate);
        assert_eq!(disposition(&record), "awaiting_required_roles");
    }

    #[test]
    fn source_frontier_change_stales_prior_review_attestations() {
        let mut record = base_record();
        record["subject"]["source_frontier_digest"] = json!("9".repeat(64));
        assert_eq!(disposition(&record), "stale_subject_or_policy");
    }

    #[test]
    fn approval_for_a_different_assertion_scope_is_not_reused() {
        let mut record = base_record();
        record["review_events"][1]["assertion_pair_ids"] =
            json!(["risk:ap:assertion:completeness"]);
        assert_eq!(disposition(&record), "review_scope_mismatch");
    }

    #[test]
    fn policy_change_without_rebound_subject_or_attestations_is_stale() {
        let mut record = base_record();
        record["review_policy"]["required_role_counts"][0]["minimum_unique_principals"] = json!(2);
        assert_eq!(disposition(&record), "stale_subject_or_policy");
    }

    #[test]
    fn adverse_review_decision_blocks_structural_readiness() {
        let mut record = base_record();
        record["review_events"][1]["decision"] = json!("request_changes");
        assert_eq!(disposition(&record), "blocked_by_adverse_review");
    }

    #[test]
    fn duplicated_event_identity_is_not_counted_twice() {
        let mut record = base_record();
        let mut duplicate = record["review_events"][1].clone();
        duplicate["principal_ref"] = json!("principal:second-approver");
        record["review_events"]
            .as_array_mut()
            .unwrap()
            .push(duplicate);
        assert_eq!(disposition(&record), "blocked_by_review_contract");
    }

    #[test]
    fn wrong_role_decision_is_rejected() {
        let mut record = base_record();
        record["review_events"][0]["decision"] = json!("approved");
        assert_eq!(disposition(&record), "blocked_by_review_contract");
    }

    #[test]
    fn canonical_subject_digest_is_stable_under_object_key_order() {
        let first: Value = serde_json::from_str(r#"{"a":1,"b":{"x":2,"y":3}}"#).unwrap();
        let second: Value = serde_json::from_str(r#"{"b":{"y":3,"x":2},"a":1}"#).unwrap();
        assert_eq!(
            digest_value(SUBJECT_DOMAIN, &first),
            digest_value(SUBJECT_DOMAIN, &second)
        );
    }

    #[test]
    fn array_order_remains_semantic_in_subject_commitment() {
        let first = json!({"assertions":["a","b"]});
        let second = json!({"assertions":["b","a"]});
        assert_ne!(
            digest_value(SUBJECT_DOMAIN, &first),
            digest_value(SUBJECT_DOMAIN, &second)
        );
    }
    #[test]
    fn generated_report_satisfies_closed_report_schema() {
        let report = verify_value(&base_record());
        let schema: Value = serde_json::from_str(include_str!("../report.schema.json")).unwrap();
        let validator = draft202012::options().build(&schema).unwrap();
        assert!(validator.is_valid(&report));
    }

    #[test]
    fn input_json_and_candidate_schema_are_checked_before_semantics() {
        let record = base_record();
        let input = serde_json::to_vec(&record).unwrap();
        let schema = include_bytes!("../../FIN_ACCOUNTING_REVIEW_V1.0.0.schema.json");
        let report = verify(&input, schema);
        assert_eq!(
            report["disposition"],
            "structurally_ready_pending_external_authority"
        );
    }
}
