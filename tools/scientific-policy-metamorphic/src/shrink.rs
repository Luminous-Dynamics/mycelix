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
    pub source_input_sha256: String,
    pub original_value_sha256: String,
    pub minimized_value_sha256: String,
    pub reduction_profile: String,
    pub input_encoding_profile: String,
    pub protected_paths: Vec<String>,
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
enum ReductionKind {
    RemoveObjectMember,
    RemoveArrayElement,
    ReplaceValue(Value),
}

#[derive(Debug, Clone)]
struct Reduction {
    path: String,
    operation: String,
    kind: ReductionKind,
}

fn stable_json_bytes(value: &Value, output: &mut Vec<u8>) -> Result<(), ShrinkError> {
    match value {
        Value::Object(object) => {
            let mut keys = object.keys().collect::<Vec<_>>();
            keys.sort();
            output.push(b'{');
            for (index, key) in keys.iter().enumerate() {
                if index > 0 {
                    output.push(b',');
                }
                let encoded_key = serde_json::to_vec(key.as_str())
                    .map_err(|error| ShrinkError(format!("serialize JSON key: {error}")))?;
                output.extend_from_slice(&encoded_key);
                output.push(b':');
                if let Some(child) = object.get(*key) {
                    stable_json_bytes(child, output)?;
                }
            }
            output.push(b'}');
        }
        Value::Array(values) => {
            output.push(b'[');
            for (index, child) in values.iter().enumerate() {
                if index > 0 {
                    output.push(b',');
                }
                stable_json_bytes(child, output)?;
            }
            output.push(b']');
        }
        _ => {
            let encoded = serde_json::to_vec(value)
                .map_err(|error| ShrinkError(format!("serialize JSON value: {error}")))?;
            output.extend_from_slice(&encoded);
        }
    }
    Ok(())
}

fn sha256_bytes(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

fn sha256(value: &Value) -> Result<String, ShrinkError> {
    let mut bytes = Vec::new();
    stable_json_bytes(value, &mut bytes)?;
    Ok(sha256_bytes(&bytes))
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

fn push_reduction(
    output: &mut Vec<Reduction>,
    reduction: Reduction,
    limit: usize,
) -> bool {
    if output.len() >= limit {
        return false;
    }
    output.push(reduction);
    true
}

fn collect_reductions(
    current: &Value,
    path: &str,
    protected_paths: &BTreeSet<String>,
    output: &mut Vec<Reduction>,
    limit: usize,
) -> bool {
    if is_protected(path, protected_paths) {
        return false;
    }

    match current {
        Value::Object(object) => {
            let mut keys = object.keys().cloned().collect::<Vec<_>>();
            keys.sort();
            for key in &keys {
                let child_path = format!("{path}/{}", escape_pointer_token(key));
                if is_protected(&child_path, protected_paths) {
                    continue;
                }
                if !push_reduction(
                    output,
                    Reduction {
                        path: child_path,
                        operation: "remove_object_member".to_owned(),
                        kind: ReductionKind::RemoveObjectMember,
                    },
                    limit,
                ) {
                    return true;
                }
            }
            for key in &keys {
                if let Some(value) = object.get(key) {
                    let child_path = format!("{path}/{}", escape_pointer_token(key));
                    if collect_reductions(value, &child_path, protected_paths, output, limit) {
                        return true;
                    }
                }
            }
        }
        Value::Array(values) => {
            for index in (0..values.len()).rev() {
                if !push_reduction(
                    output,
                    Reduction {
                        path: format!("{path}/{index}"),
                        operation: "remove_array_element".to_owned(),
                        kind: ReductionKind::RemoveArrayElement,
                    },
                    limit,
                ) {
                    return true;
                }
            }
            for (index, value) in values.iter().enumerate() {
                let child_path = format!("{path}/{index}");
                if collect_reductions(value, &child_path, protected_paths, output, limit) {
                    return true;
                }
            }
        }
        Value::String(value) => {
            let empty = Value::String(String::new());
            if &empty != current
                && !push_reduction(
                    output,
                    Reduction {
                        path: path.to_owned(),
                        operation: "replace_string_with_empty".to_owned(),
                        kind: ReductionKind::ReplaceValue(empty),
                    },
                    limit,
                )
            {
                return true;
            }
            if let Some(first) = value.chars().next() {
                let shortened = Value::String(first.to_string());
                if &shortened != current
                    && !push_reduction(
                        output,
                        Reduction {
                            path: path.to_owned(),
                            operation: "shorten_string_to_first_scalar".to_owned(),
                            kind: ReductionKind::ReplaceValue(shortened),
                        },
                        limit,
                    )
                {
                    return true;
                }
            }
        }
        Value::Number(_) => {
            let zero = Value::Number(Number::from(0));
            if &zero != current
                && !push_reduction(
                    output,
                    Reduction {
                        path: path.to_owned(),
                        operation: "replace_number_with_zero".to_owned(),
                        kind: ReductionKind::ReplaceValue(zero),
                    },
                    limit,
                )
            {
                return true;
            }
        }
        Value::Bool(value) => {
            if *value
                && !push_reduction(
                    output,
                    Reduction {
                        path: path.to_owned(),
                        operation: "replace_true_with_false".to_owned(),
                        kind: ReductionKind::ReplaceValue(Value::Bool(false)),
                    },
                    limit,
                )
            {
                return true;
            }
        }
        Value::Null => {}
    }
    false
}

fn encode_reductions(
    value: &Value,
    protected_paths: &BTreeSet<String>,
    limit: usize,
) -> (Vec<Reduction>, bool) {
    let mut output = Vec::new();
    let truncated = collect_reductions(value, "", protected_paths, &mut output, limit);
    (output, truncated)
}

fn decode_pointer_token(token: &str) -> String {
    token.replace("~1", "/").replace("~0", "~")
}

fn apply_reduction(
    current: &Value,
    reduction: &Reduction,
) -> Result<Value, ShrinkError> {
    let mut candidate = current.clone();
    match &reduction.kind {
        ReductionKind::ReplaceValue(value) => {
            if reduction.path.is_empty() {
                candidate = value.clone();
            } else {
                let target = candidate
                    .pointer_mut(&reduction.path)
                    .ok_or_else(|| ShrinkError(format!("reduction target disappeared: {}", reduction.path)))?;
                *target = value.clone();
            }
        }
        ReductionKind::RemoveObjectMember | ReductionKind::RemoveArrayElement => {
            let (parent_path, token) = reduction
                .path
                .rsplit_once('/')
                .ok_or_else(|| ShrinkError(format!("invalid removal pointer: {}", reduction.path)))?;
            let decoded = decode_pointer_token(token);
            let parent = candidate
                .pointer_mut(parent_path)
                .ok_or_else(|| ShrinkError(format!("reduction parent disappeared: {parent_path}")))?;
            match &reduction.kind {
                ReductionKind::RemoveObjectMember => {
                    let object = parent.as_object_mut().ok_or_else(|| {
                        ShrinkError(format!("object reduction parent is not an object: {parent_path}"))
                    })?;
                    if object.remove(&decoded).is_none() {
                        return Err(ShrinkError(format!("object reduction key disappeared: {decoded}")));
                    }
                }
                ReductionKind::RemoveArrayElement => {
                    let index = decoded.parse::<usize>().map_err(|_| {
                        ShrinkError(format!("array reduction index is invalid: {decoded}"))
                    })?;
                    let array = parent.as_array_mut().ok_or_else(|| {
                        ShrinkError(format!("array reduction parent is not an array: {parent_path}"))
                    })?;
                    if index >= array.len() {
                        return Err(ShrinkError(format!("array reduction index is out of range: {index}")));
                    }
                    array.remove(index);
                }
                ReductionKind::ReplaceValue(_) => unreachable!(),
            }
        }
    }
    Ok(candidate)
}

/// Deterministically shrink an input while preserving a caller-provided
/// predicate. A complete result is 1-minimal in the finite neighborhood
/// defined by collect_reductions, not a claim of global minimum.
pub fn minimize_json<P>(
    input: &Value,
    source_input_bytes: &[u8],
    predicate_id: &str,
    protected_paths: &BTreeSet<String>,
    max_candidate_evaluations: usize,
    mut predicate: P,
) -> Result<ShrinkResult, ShrinkError>
where
    P: FnMut(&Value) -> Result<bool, String>,
{
    let decoded_source: Value = serde_json::from_slice(source_input_bytes)
        .map_err(|error| ShrinkError(format!("parse exact source input bytes: {error}")))?;
    if decoded_source != *input {
        return Err(ShrinkError(
            "exact source input bytes do not decode to the supplied JSON value".to_owned(),
        ));
    }
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

    let source_input_sha256 = sha256_bytes(source_input_bytes);
    let original_value_sha256 = sha256(input)?;
    let mut current = input.clone();
    let mut candidates = Vec::new();
    let mut accepted_reductions = 0usize;
    let mut complete = false;
    let mut is_one_minimal = false;

    'shrink: loop {
        let remaining_budget = max_candidate_evaluations - candidates.len();
        let (reductions, truncated) =
            encode_reductions(&current, protected_paths, remaining_budget);
        let before_sha256 = sha256(&current)?;
        let mut accepted = false;

        for reduction in reductions {
            let candidate = apply_reduction(&current, &reduction)?;
            let candidate_sha256 = sha256(&candidate)?;
            let evaluation = predicate(&candidate);
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
                current = candidate;
                accepted_reductions += 1;
                accepted = true;
                break;
            }
        }

        if accepted {
            continue;
        }
        if truncated {
            break 'shrink;
        }

        complete = true;
        is_one_minimal = true;
        break;
    }

    let minimized_value_sha256 = sha256(&current)?;
    Ok(ShrinkResult {
        minimized_value: current,
        trace: ShrinkTrace {
            schema_version: "mycelix-policy-counterexample-shrink-v0.1".to_owned(),
            predicate_id: predicate_id.to_owned(),
            source_input_sha256,
            original_value_sha256,
            minimized_value_sha256,
            reduction_profile: "json-object-array-scalar-v0.1".to_owned(),
            input_encoding_profile: "serde-json-sorted-objects-v0.1".to_owned(),
            protected_paths: protected_paths.iter().cloned().collect(),
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
        let source_bytes = serde_json::to_vec(&input).unwrap();
        let result = minimize_json(
            &input,
            &source_bytes,
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
        let source_bytes = serde_json::to_vec(&input).unwrap();
        let run = || {
            minimize_json(
                &input,
                &source_bytes,
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
        let source_bytes = serde_json::to_vec(&input).unwrap();
        let result = minimize_json(
            &input,
            &source_bytes,
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
    fn predicate_errors_during_candidate_evaluation_return_incomplete_trace() {
        let input = serde_json::json!({
            "principal": "actor:fixed",
            "action": "claim_proposed",
            "resource": "claim:fixed",
            "context": {"required": true, "noise": true}
        });
        let mut calls = 0usize;
        let source_bytes = serde_json::to_vec(&input).unwrap();
        let result = minimize_json(
            &input,
            &source_bytes,
            "error-control",
            &protected_request_paths(),
            100,
            |_| {
                calls += 1;
                if calls == 1 {
                    Ok(true)
                } else {
                    Err("engine process indeterminate".to_owned())
                }
            },
        )
        .unwrap();
        assert!(!result.trace.complete);
        assert!(!result.trace.is_one_minimal);
        assert_eq!(result.trace.candidate_evaluations, 1);
        assert_eq!(
            result.trace.candidates[0].evaluation_error.as_deref(),
            Some("engine process indeterminate")
        );
    }

    #[test]
    fn exact_source_bytes_and_normalized_value_have_distinct_identities() {
        let first_bytes = br#"{"a":1,"b":2}"#;
        let second_bytes = br#"{ "b": 2, "a": 1 }"#;
        let first_value: Value = serde_json::from_slice(first_bytes).unwrap();
        let second_value: Value = serde_json::from_slice(second_bytes).unwrap();
        let protected = ["/a", "/b"]
            .into_iter()
            .map(str::to_owned)
            .collect::<BTreeSet<_>>();

        let first = minimize_json(
            &first_value,
            first_bytes,
            "source-identity-control",
            &protected,
            100,
            |_| Ok(true),
        )
        .unwrap();
        let second = minimize_json(
            &second_value,
            second_bytes,
            "source-identity-control",
            &protected,
            100,
            |_| Ok(true),
        )
        .unwrap();

        assert_ne!(
            first.trace.source_input_sha256,
            second.trace.source_input_sha256
        );
        assert_eq!(
            first.trace.original_value_sha256,
            second.trace.original_value_sha256
        );
    }

    #[test]
    fn rejects_source_bytes_that_do_not_match_the_supplied_value() {
        let input = serde_json::json!({"a": 1});
        assert!(minimize_json(
            &input,
            br#"{"a":2}"#,
            "source-mismatch-control",
            &BTreeSet::new(),
            100,
            |_| Ok(true),
        )
        .is_err());
    }

    #[test]
    fn stable_digest_ignores_object_member_insertion_order() {
        let first: Value = serde_json::from_str(r#"{"b":2,"a":{"y":4,"x":3}}"#).unwrap();
        let second: Value = serde_json::from_str(r#"{"a":{"x":3,"y":4},"b":2}"#).unwrap();
        assert_eq!(sha256(&first).unwrap(), sha256(&second).unwrap());
    }

    #[test]
    fn numeric_simplification_is_monotone_and_terminates_at_zero() {
        let input = serde_json::json!({"value": 99});
        let source_bytes = serde_json::to_vec(&input).unwrap();
        let result = minimize_json(
            &input,
            &source_bytes,
            "numeric-reduction",
            &BTreeSet::new(),
            100,
            |_| Ok(true),
        )
        .unwrap();
        assert_eq!(result.minimized_value, serde_json::json!({"value": 0}));
        assert!(result.trace.complete);
        assert!(result.trace.is_one_minimal);
    }

    #[test]
    fn accepts_only_inputs_that_initially_satisfy_predicate() {
        let input = serde_json::json!({"principal": "wrong"});
        let source_bytes = serde_json::to_vec(&input).unwrap();
        assert!(minimize_json(
            &input,
            &source_bytes,
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
        let source_bytes = serde_json::to_vec(&input).unwrap();
        let result = minimize_json(&input, &source_bytes, "nested-resource-guard", &protected, 10_000, predicate)
            .unwrap();
        assert_eq!(result.minimized_value["resource"], input["resource"]);
        assert!(result.trace.is_one_minimal);
    }
}
