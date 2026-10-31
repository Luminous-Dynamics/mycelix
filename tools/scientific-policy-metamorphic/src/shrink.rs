//! Deterministic JSON counterexample minimization with a traceable 1-minimality claim.
//!
//! This module is generic: callers provide the exact predicate to preserve.
//! It does not interpret authorization policy semantics or claim that a
//! minimized input is correct for any particular policy engine.

use serde::Serialize;
use serde_json::{Number, Value};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ShrinkError(pub String);

impl fmt::Display for ShrinkError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl std::error::Error for ShrinkError {}

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct CandidateTrace {
    pub sequence: usize,
    pub path: String,
    pub operation: String,
    pub before_sha256: String,
    pub candidate_sha256: String,
    pub predicate_preserved: Option<bool>,
    pub accepted: bool,
    pub evaluation_error: Option<String>,
}

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct ShrinkTrace {
    pub schema_version: String,
    pub predicate_id: String,
    pub original_sha256: String,
    pub minimized_sha256: String,
    pub max_candidate_evaluations: usize,
    pub candidate_evaluations: usize,
    pub accepted_reductions: usize,
    pub complete: bool,
    pub is_one_minimal: bool,
    pub claim_ceiling: String,
    pub candidates: Vec<CandidateTrace>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ShrinkResult {
    pub minimized_value: Value,
    pub trace: ShrinkTrace,
}

#[derive(Debug, Clone)]
struct Reduction {
    path: String,
    operation: String,
    value: Value,
}

fn sha256(value: &Value) -> Result<String, ShrinkError> {
    let bytes = serde_json::to_vec(value)
        .map_err(|error| ShrinkError(format!("serialize shrink candidate: {error}")))?;
    Ok(hex::encode(Sha256::digest(bytes)))
}

fn escape_pointer_token(token: &str) -> String {
    token.replace('~', "~0").replace('/', "~1")
}

fn is_protected(path: &str, protected_paths: &BTreeSet<String>) -> bool {
    protected_paths.iter().any(|protected| {
        path == protected
            || path
                .strip_prefix(protected)
                .is_some_and(|suffix| suffix.starts_with('/'))
    })
}

fn push_if_changed(
    output: &mut Vec<Reduction>,
    path: String,
    operation: &str,
    candidate: Value,
    current: &Value,
) {
    if &candidate != current {
        output.push(Reduction {
            path,
            operation: operation.to_owned(),
            value: candidate,
        });
    }
}

fn collect_reductions(
    current: &Value,
    path: &str,
    protected_paths: &BTreeSet<String>,
    output: &mut Vec<Reduction>,
) {
    if is_protected(path, protected_paths) {
        return;
    }

    match current {
        Value::Object(object) => {
            for key in object.keys() {
                let child_path = format!("{path}/{}", escape_pointer_token(key));
                if is_protected(&child_path, protected_paths) {
                    continue;
                }
                let mut candidate = object.clone();
                candidate.remove(key);
                push_if_changed(
                    output,
                    child_path,
                    "remove_object_member",
                    Value::Object(candidate),
                    current,
                );
            }
            for (key, value) in object {
                let child_path = format!("{path}/{}", escape_pointer_token(key));
                collect_reductions(value, &child_path, protected_paths, output);
            }
        }
        Value::Array(values) => {
            for index in (0..values.len()).rev() {
                let mut candidate = values.clone();
                candidate.remove(index);
                push_if_changed(
                    output,
                    format!("{path}/{index}"),
                    "remove_array_element",
                    Value::Array(candidate),
                    current,
                );
            }
            for (index, value) in values.iter().enumerate() {
                let child_path = format!("{path}/{index}");
                collect_reductions(value, &child_path, protected_paths, output);
            }
        }
        Value::String(value) => {
            push_if_changed(
                output,
                path.to_owned(),
                "replace_string_with_empty",
                Value::String(String::new()),
                current,
            );
            if let Some(first) = value.chars().next() {
                push_if_changed(
                    output,
                    path.to_owned(),
                    "shorten_string_to_first_scalar",
                    Value::String(first.to_string()),
                    current,
                );
            }
        }
        Value::Number(_) => {
            push_if_changed(
                output,
                path.to_owned(),
                "replace_number_with_zero",
                Value::Number(Number::from(0)),
                current,
            );
            push_if_changed(
                output,
                path.to_owned(),
                "replace_number_with_one",
                Value::Number(Number::from(1)),
                current,
            );
        }
        Value::Bool(value) => {
            if *value {
                push_if_changed(
                    output,
                    path.to_owned(),
                    "replace_true_with_false",
                    Value::Bool(false),
                    current,
                );
            }
        }
        Value::Null => {}
    }
}

fn encode_reductions(
    value: &Value,
    protected_paths: &BTreeSet<String>,
) -> Vec<Reduction> {
    let mut output = Vec::new();
    collect_reductions(value, "", protected_paths, &mut output);
    output
}

/// Deterministically shrink an input while preserving a caller-provided
/// predicate. A complete result is 1-minimal in the finite neighborhood
/// defined by collect_reductions, not a claim of global minimum.
pub fn minimize_json<P>(
    input: &Value,
    predicate_id: &str,
    protected_paths: &BTreeSet<String>,
    max_candidate_evaluations: usize,
    mut predicate: P,
) -> Result<ShrinkResult, ShrinkError>
where
    P: FnMut(&Value) -> Result<bool, String>,
{
    if predicate_id.trim().is_empty() {
        return Err(ShrinkError("predicate_id must not be empty".to_owned()));
    }
    if max_candidate_evaluations == 0 {
        return Err(ShrinkError(
            "max_candidate_evaluations must be greater than zero".to_owned(),
        ));
    }
    if !predicate(input).map_err(|error| {
        ShrinkError(format!("initial predicate evaluation failed: {error}"))
    })? {
        return Err(ShrinkError(
            "initial input does not satisfy the preserved predicate".to_owned(),
        ));
    }

    let original_sha256 = sha256(input)?;
    let mut current = input.clone();
    let mut candidates = Vec::new();
    let mut accepted_reductions = 0usize;
    let mut complete = false;
    let mut is_one_minimal = false;

    'shrink: loop {
        let reductions = encode_reductions(&current, protected_paths);
        let before_sha256 = sha256(&current)?;
        let mut accepted = false;

        for reduction in reductions {
            if candidates.len() >= max_candidate_evaluations {
                break 'shrink;
            }
            let candidate_sha256 = sha256(&reduction.value)?;
            let evaluation = predicate(&reduction.value);
            let (predicate_preserved, accepted_candidate, evaluation_error) =
                match evaluation {
                    Ok(preserved) => (Some(preserved), preserved, None),
                    Err(error) => (None, false, Some(error)),
                };
            candidates.push(CandidateTrace {
                sequence: candidates.len(),
                path: reduction.path,
                operation: reduction.operation,
                before_sha256: before_sha256.clone(),
                candidate_sha256,
                predicate_preserved,
                accepted: accepted_candidate,
                evaluation_error: evaluation_error.clone(),
            });

            if evaluation_error.is_some() {
                break 'shrink;
            }
            if accepted_candidate {
                current = reduction.value;
                accepted_reductions += 1;
                accepted = true;
                break;
            }
        }

        if accepted {
            continue;
        }

        complete = true;
        is_one_minimal = true;
        break;
    }

    let minimized_sha256 = sha256(&current)?;
    Ok(ShrinkResult {
        minimized_value: current,
        trace: ShrinkTrace {
            schema_version: "mycelix-policy-counterexample-shrink-v0.1".to_owned(),
            predicate_id: predicate_id.to_owned(),
            original_sha256,
            minimized_sha256,
            max_candidate_evaluations,
            candidate_evaluations: candidates.len(),
            accepted_reductions,
            complete,
            is_one_minimal,
            claim_ceiling: concat!(
                "deterministic 1-minimality in the recorded finite JSON reduction neighborhood; ",
                "not global minimality, policy correctness, formal equivalence or Rust authorization parity"
            )
            .to_owned(),
            candidates,
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn protected_request_paths() -> BTreeSet<String> {
        ["/principal", "/action", "/resource"]
            .into_iter()
            .map(str::to_owned)
            .collect()
    }

    fn mismatch_predicate(value: &Value) -> Result<bool, String> {
        Ok(value.get("principal").and_then(Value::as_str) == Some("actor:fixed")
            && value.get("action").and_then(Value::as_str) == Some("claim_proposed")
            && value.get("resource").and_then(Value::as_str) == Some("claim:fixed")
            && value
                .get("context")
                .and_then(|context| context.get("required"))
                .and_then(Value::as_bool)
                == Some(true))
    }

    #[test]
    fn shrinks_noise_and_preserves_protected_authorization_identity() {
        let input = serde_json::json!({
            "principal": "actor:fixed",
            "action": "claim_proposed",
            "resource": "claim:fixed",
            "context": {
                "required": true,
                "irrelevant": "a long irrelevant value",
                "debug": {"nested": [1, 2, 3]}
            },
            "extra_entities": [
                {"uid": "actor:noise-1", "roles": ["viewer"]},
                {"uid": "actor:noise-2", "roles": ["reader"]}
            ]
        });
        let result = minimize_json(
            &input,
            "test-policy-decision-divergence",
            &protected_request_paths(),
            10_000,
            mismatch_predicate,
        )
        .expect("minimization succeeds");

        assert!(result.trace.complete);
        assert!(result.trace.is_one_minimal);
        assert!(result.trace.accepted_reductions > 0);
        assert!(mismatch_predicate(&result.minimized_value).unwrap());
        for key in ["principal", "action", "resource"] {
            assert_eq!(result.minimized_value[key], input[key]);
        }
        assert!(result.minimized_value.get("extra_entities").is_none());
        assert_eq!(result.minimized_value["context"]["required"], true);
    }

    #[test]
    fn trace_is_deterministic_for_identical_inputs_and_predicates() {
        let input = serde_json::json!({
            "principal": "actor:fixed",
            "action": "claim_proposed",
            "resource": "claim:fixed",
            "context": {"required": true, "unused": [1, 2, 3]}
        });
        let run = || {
            minimize_json(
                &input,
                "deterministic-predicate",
                &protected_request_paths(),
                10_000,
                mismatch_predicate,
            )
            .unwrap()
        };
        let first = run();
        let second = run();
        assert_eq!(first.minimized_value, second.minimized_value);
        assert_eq!(first.trace, second.trace);
    }

    #[test]
    fn candidate_budget_exhaustion_never_claims_minimality() {
        let input = serde_json::json!({
            "principal": "actor:fixed",
            "action": "claim_proposed",
            "resource": "claim:fixed",
            "context": {"required": true, "noise": true}
        });
        let result = minimize_json(
            &input,
            "budget-control",
            &protected_request_paths(),
            1,
            mismatch_predicate,
        )
        .unwrap();
        assert!(!result.trace.complete);
        assert!(!result.trace.is_one_minimal);
    }

    #[test]
    fn predicate_errors_never_claim_minimality() {
        let input = serde_json::json!({
            "principal": "actor:fixed",
            "action": "claim_proposed",
            "resource": "claim:fixed",
            "context": {"required": true, "noise": true}
        });
        let result = minimize_json(
            &input,
            "error-control",
            &protected_request_paths(),
            100,
            |_| Err("engine process indeterminate".to_owned()),
        );
        assert!(result.is_err());
    }

    #[test]
    fn accepts_only_inputs_that_initially_satisfy_predicate() {
        let input = serde_json::json!({"principal": "wrong"});
        assert!(minimize_json(
            &input,
            "initial-false-control",
            &BTreeSet::new(),
            100,
            |_| Ok(false)
        )
        .is_err());
    }

    #[test]
    fn refuses_to_change_protected_nested_resource() {
        let input = serde_json::json!({
            "principal": "actor:fixed",
            "action": "claim_proposed",
            "resource": {"id": "claim:fixed", "exists": false, "noise": true},
            "context": {"required": true}
        });
        let protected = ["/principal", "/action", "/resource"]
            .into_iter()
            .map(str::to_owned)
            .collect::<BTreeSet<_>>();
        let predicate = |value: &Value| {
            Ok(value["principal"] == "actor:fixed"
                && value["action"] == "claim_proposed"
                && value["resource"]["exists"] == false
                && value["context"]["required"] == true)
        };
        let result = minimize_json(&input, "nested-resource-guard", &protected, 10_000, predicate)
            .unwrap();
        assert_eq!(result.minimized_value["resource"], input["resource"]);
        assert!(result.trace.is_one_minimal);
    }
}
