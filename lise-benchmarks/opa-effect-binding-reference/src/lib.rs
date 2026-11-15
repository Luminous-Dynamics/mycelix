#![forbid(unsafe_code)]

//! Independent OPA decision-receipt and protected-effect binding reference.
//!
//! OPA decision logs describe policy queries; this evaluator deliberately
//! treats them as evidence of a decision, not proof that an external mutation
//! occurred. A separate effect observation must bind to the exact decision
//! and exact operation before the evaluator reports reconciliation.
//!
//! The included receipt/effect objects are synthetic and untrusted. This is
//! not an OPA plugin, policy enforcement point, provider adapter, or production
//! trust verifier.

use serde_json::{json, Value};
use sha2::{Digest, Sha256};

pub const SUITE_ID: &str = "LISE-OPA-EFFECT-BINDING-v0.1";
pub const EVALUATOR_VERSION: &str = "opa-effect-binding-reference-v0.1";

fn string<'a>(value: &'a Value, key: &str, context: &str) -> Result<&'a str, String> {
    value
        .get(key)
        .and_then(Value::as_str)
        .filter(|s| !s.trim().is_empty())
        .ok_or_else(|| format!("{context}: '{key}' must be a nonempty string"))
}

fn optional_object<'a>(value: &'a Value, key: &str) -> Option<&'a Value> {
    value.get(key).filter(|item| item.is_object())
}

fn array<'a>(value: &'a Value, key: &str, context: &str) -> Result<&'a Vec<Value>, String> {
    value
        .get(key)
        .and_then(Value::as_array)
        .ok_or_else(|| format!("{context}: '{key}' must be an array"))
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
        || !b.iter().enumerate().all(|(i, c)| {
            matches!(i, 4 | 7 | 10 | 13 | 16 | 19) || c.is_ascii_digit()
        })
    {
        return Err(format!("{context}: '{key}' must be YYYY-MM-DDTHH:MM:SSZ"));
    }
    Ok(result)
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|c| c.is_ascii_hexdigit())
}

fn report(
    overall: &str,
    decision_state: &str,
    effect_state: &str,
    reason: &str,
    blockers: Vec<String>,
) -> Value {
    json!({
        "suite_id": SUITE_ID,
        "evaluator_version": EVALUATOR_VERSION,
        "overall": overall,
        "decision_state": decision_state,
        "effect_state": effect_state,
        "reason_code": reason,
        "blockers": blockers,
        "dispatch_eligible": false,
        "claim_ceiling": "Synthetic OPA decision receipt and effect binding only; no production log authentication, PEP enforcement, provider effect, or independent source trust is established."
    })
}

fn indeterminate(reason: &str) -> Value {
    report(
        "INDETERMINATE",
        "UNKNOWN",
        "UNKNOWN",
        reason,
        vec![reason.to_owned()],
    )
}

fn evaluate_inner(fixture: &Value) -> Result<Value, String> {
    if fixture.get("synthetic").and_then(Value::as_bool) != Some(true) {
        return Ok(indeterminate("fixture_not_marked_synthetic"));
    }
    let known_at = timestamp(fixture, "evaluation_known_at", "fixture")?;
    let intent = fixture.get("intent").ok_or("fixture: missing intent")?;
    let principal = string(intent, "principal_ref", "intent")?;
    let action = string(intent, "action", "intent")?;
    let purpose = string(intent, "purpose", "intent")?;
    let resource = string(intent, "resource_ref", "intent")?;
    let operation_digest = string(intent, "operation_digest", "intent")?;
    let policy_digest = string(intent, "policy_digest", "intent")?;
    let expected_path = string(intent, "policy_path", "intent")?;
    let expected_bundle_name = string(intent, "expected_bundle_name", "intent")?;
    let expected_bundle_revision = string(intent, "expected_bundle_revision", "intent")?;

    let log = fixture.get("opa_decision_log").ok_or("fixture: missing opa_decision_log")?;
    let decision_id = string(log, "decision_id", "OPA decision log")?;
    let trace_id = string(log, "trace_id", "OPA decision log")?;
    let _span_id = string(log, "span_id", "OPA decision log")?;
    let path = string(log, "path", "OPA decision log")?;
    let _query = string(log, "query", "OPA decision log")?;
    let decision_at = timestamp(log, "timestamp", "OPA decision log")?;
    if decision_at > known_at {
        return Ok(indeterminate("decision_not_yet_known_at_evaluation_cutoff"));
    }
    if path != expected_path {
        return Ok(report(
            "DECISION_SCOPE_MISMATCH",
            "UNKNOWN",
            "UNKNOWN",
            "opa_policy_path_mismatch",
            vec!["OPA decision path differs from the intent-bound policy path.".to_owned()],
        ));
    }

    let decision_result = log
        .get("result")
        .and_then(Value::as_bool)
        .ok_or("OPA decision log: result must be boolean")?;
    let input = log.get("input").ok_or("OPA decision log: missing input")?;
    let input_matches = [
        ("principal_ref", principal),
        ("action", action),
        ("purpose", purpose),
        ("resource_ref", resource),
        ("operation_digest", operation_digest),
        ("policy_digest", policy_digest),
    ]
    .iter()
    .all(|(key, expected)| input.get(*key).and_then(Value::as_str) == Some(expected));
    if !input_matches {
        return Ok(report(
            "DECISION_SCOPE_MISMATCH",
            if decision_result { "POLICY_ALLOWED" } else { "POLICY_DENIED" },
            "UNKNOWN",
            "opa_input_does_not_match_exact_intent",
            vec!["OPA input differs from principal/action/purpose/resource/operation/policy in the accepted intent.".to_owned()],
        ));
    }

    let erased = array(log, "erased", "OPA decision log")?;
    let masked = array(log, "masked", "OPA decision log")?;
    if !erased.is_empty() || !masked.is_empty() {
        return Ok(indeterminate("opa_input_contains_erased_or_masked_fields"));
    }

    let bundles = log.get("bundles").ok_or("OPA decision log: missing bundles")?;
    let revision = bundles
        .get(expected_bundle_name)
        .and_then(|bundle| bundle.get("revision"))
        .and_then(Value::as_str)
        .ok_or("OPA decision log: expected bundle revision is missing")?;
    if revision != expected_bundle_revision {
        return Ok(indeterminate("opa_bundle_revision_mismatch"));
    }

    let input_bytes = serde_json::to_vec(input).map_err(|e| e.to_string())?;
    let input_digest = sha256(&input_bytes);
    let receipt = fixture
        .get("collection_receipt")
        .ok_or("fixture: missing collection_receipt")?;
    if string(receipt, "source_kind", "collection receipt")? != "opa_decision_log"
        || string(receipt, "decision_id", "collection receipt")? != decision_id
        || string(receipt, "trace_id", "collection receipt")? != trace_id
        || string(receipt, "input_sha256", "collection receipt")? != input_digest
        || string(receipt, "bundle_revision", "collection receipt")? != revision
        || string(receipt, "completeness_state", "collection receipt")?
            != "complete_for_decision"
        || string(receipt, "signature_state", "collection receipt")? != "synthetic_unverified"
        || string(receipt, "trust_state", "collection receipt")? != "untrusted_synthetic"
        || !is_sha256(string(receipt, "adapter_digest", "collection receipt")?)
    {
        return Ok(indeterminate("opa_collection_receipt_invalid_or_mismatched"));
    }
    let _receipt_ref = string(receipt, "receipt_ref", "collection receipt")?;
    let _source_identity = string(receipt, "source_identity", "collection receipt")?;
    let receipt_observed_at = timestamp(receipt, "observed_at", "collection receipt")?;
    if receipt_observed_at < decision_at || receipt_observed_at > known_at {
        return Ok(indeterminate("opa_collection_receipt_outside_knowledge_window"));
    }

    let decision_state = if decision_result {
        "POLICY_ALLOWED"
    } else {
        "POLICY_DENIED"
    };

    let Some(effect) = optional_object(fixture, "effect_observation") else {
        return Ok(report(
            if decision_result { "POLICY_ALLOW_ONLY" } else { "POLICY_DENIED" },
            decision_state,
            "NO_EFFECT_OBSERVATION",
            if decision_result {
                "policy_allow_is_not_effect_proof"
            } else {
                "policy_denied_no_effect_observed"
            },
            if decision_result {
                vec!["OPA records an allow decision, but no independent protected-resource observation was supplied.".to_owned()]
            } else {
                Vec::new()
            },
        ));
    };

    let effect_decision_id = string(effect, "decision_id", "effect observation")?;
    let effect_digest = string(effect, "operation_digest", "effect observation")?;
    let effect_resource = string(effect, "resource_ref", "effect observation")?;
    let effect_action = string(effect, "action", "effect observation")?;
    let effect_purpose = string(effect, "purpose", "effect observation")?;
    if effect_decision_id != decision_id
        || effect_digest != operation_digest
        || effect_resource != resource
        || effect_action != action
        || effect_purpose != purpose
    {
        return Ok(report(
            "EFFECT_BINDING_MISMATCH",
            decision_state,
            "UNBOUND_EFFECT",
            "effect_observation_does_not_match_exact_decision_operation",
            vec!["Effect observation is not bound to the exact decision ID and intent operation.".to_owned()],
        ));
    }
    let effect_at = timestamp(effect, "observed_at", "effect observation")?;
    if effect_at > known_at {
        return Ok(indeterminate("effect_not_yet_observed_at_evaluation_cutoff"));
    }
    let kind = string(effect, "kind", "effect observation")?;
    let outcome = string(effect, "outcome", "effect observation")?;
    if !decision_result && matches!(outcome, "applied" | "partially_applied") {
        return Ok(report(
            "UNAUTHORIZED_EFFECT",
            decision_state,
            "EFFECT_OBSERVED",
            "effect_observed_after_policy_denial",
            vec!["A protected-resource effect was observed even though the OPA decision denied the operation.".to_owned()],
        ));
    }
    if !decision_result {
        return Ok(report(
            "POLICY_DENIED",
            decision_state,
            "NO_EFFECT_PROVEN",
            "policy_denied",
            Vec::new(),
        ));
    }
    if kind == "provider_acknowledgement" {
        return Ok(report(
            "ACKNOWLEDGED_NOT_RECONCILED",
            decision_state,
            "PROVIDER_ACK_ONLY",
            "provider_acknowledgement_is_not_independent_effect_observation",
            vec!["Provider acknowledgement is not independent evidence that the protected resource changed.".to_owned()],
        ));
    }
    if kind != "independent_resource_observation"
        || string(effect, "observation_independence", "effect observation")? != "independent"
        || string(effect, "trust_state", "effect observation")? != "fixture_only"
    {
        return Ok(report(
            "ACKNOWLEDGED_NOT_RECONCILED",
            decision_state,
            "OBSERVATION_NOT_INDEPENDENT",
            "effect_observation_independence_not_established",
            vec!["Effect evidence is not independently sourced from the policy/provider decision path.".to_owned()],
        ));
    }
    let _observer = string(effect, "observer_identity", "effect observation")?;
    let dependencies = array(effect, "source_dependencies", "effect observation")?;
    if dependencies.iter().any(|dependency| {
        dependency
            .as_str()
            .is_some_and(|value| value == "opa_decision_log" || value == "provider_acknowledgement")
    }) {
        return Ok(report(
            "ACKNOWLEDGED_NOT_RECONCILED",
            decision_state,
            "COMMON_CAUSE_NOT_EXCLUDED",
            "effect_observation_shares_policy_or_provider_source",
            vec!["The claimed independent observation shares a dependency with the policy/provider decision evidence.".to_owned()],
        ));
    }
    match outcome {
        "applied" => Ok(report(
            "EFFECT_RECONCILED",
            decision_state,
            "EFFECT_CONFIRMED_APPLIED",
            "exact_decision_and_independent_effect_observation_match",
            Vec::new(),
        )),
        "not_applied" => Ok(report(
            "EFFECT_CONFIRMED_NOT_APPLIED",
            decision_state,
            "EFFECT_CONFIRMED_NOT_APPLIED",
            "independent_observation_confirms_no_effect",
            Vec::new(),
        )),
        "partially_applied" => Ok(report(
            "PARTIAL_EFFECT_REQUIRES_RECONCILIATION",
            decision_state,
            "PARTIAL_EFFECT",
            "partial_effect_observed",
            vec!["A partial external effect requires action-level reconciliation; do not replay the whole operation.".to_owned()],
        )),
        _ => Ok(indeterminate("effect_outcome_unrecognized")),
    }
}

/// Evaluate a synthetic OPA decision log and separate protected-resource observation.
pub fn evaluate(fixture: &Value) -> Value {
    evaluate_inner(fixture).unwrap_or_else(|error| {
        report(
            "INVALID_INPUT",
            "UNKNOWN",
            "UNKNOWN",
            "input_validation_failed",
            vec![error],
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn fixture() -> Value {
        serde_json::from_str(include_str!("../fixtures/opa-effect-fixture.json"))
            .expect("fixture JSON")
    }

    fn refresh_digest(value: &mut Value) {
        let input = &value["opa_decision_log"]["input"];
        let bytes = serde_json::to_vec(input).expect("input serialization");
        value["collection_receipt"]["input_sha256"] = json!(sha256(&bytes));
    }

    #[test]
    fn frozen_fixture_reconciles_only_with_independent_effect_observation() {
        let value = fixture();
        let result = evaluate(&value);
        assert_eq!(result["overall"], "EFFECT_RECONCILED");
        assert_eq!(result["decision_state"], "POLICY_ALLOWED");
        assert_eq!(result["dispatch_eligible"], false);
    }

    #[test]
    fn opa_allow_without_effect_is_only_a_policy_decision() {
        let mut value = fixture();
        value["effect_observation"] = Value::Null;
        let result = evaluate(&value);
        assert_eq!(result["overall"], "POLICY_ALLOW_ONLY");
        assert_eq!(result["effect_state"], "NO_EFFECT_OBSERVATION");
    }

    #[test]
    fn provider_acknowledgement_is_not_independent_effect_proof() {
        let mut value = fixture();
        value["effect_observation"]["kind"] = json!("provider_acknowledgement");
        let result = evaluate(&value);
        assert_eq!(result["overall"], "ACKNOWLEDGED_NOT_RECONCILED");
    }

    #[test]
    fn denied_policy_with_no_effect_is_a_clean_denial() {
        let mut value = fixture();
        value["opa_decision_log"]["result"] = json!(false);
        value["effect_observation"] = Value::Null;
        let result = evaluate(&value);
        assert_eq!(result["overall"], "POLICY_DENIED");
    }

    #[test]
    fn effect_after_policy_denial_is_reported_as_unauthorized() {
        let mut value = fixture();
        value["opa_decision_log"]["result"] = json!(false);
        let result = evaluate(&value);
        assert_eq!(result["overall"], "UNAUTHORIZED_EFFECT");
    }

    #[test]
    fn masked_or_erased_policy_input_is_indeterminate() {
        let mut value = fixture();
        value["opa_decision_log"]["masked"] = json!(["/operation_digest"]);
        assert_eq!(evaluate(&value)["overall"], "INDETERMINATE");

        let mut value = fixture();
        value["opa_decision_log"]["erased"] = json!(["/purpose"]);
        assert_eq!(evaluate(&value)["overall"], "INDETERMINATE");
    }

    #[test]
    fn mismatched_input_digest_or_bundle_revision_is_indeterminate() {
        let mut value = fixture();
        value["collection_receipt"]["input_sha256"] = json!("0".repeat(64));
        assert_eq!(evaluate(&value)["overall"], "INDETERMINATE");

        let mut value = fixture();
        value["collection_receipt"]["bundle_revision"] = json!("old-bundle");
        assert_eq!(evaluate(&value)["overall"], "INDETERMINATE");
    }

    #[test]
    fn policy_input_must_match_the_exact_intent() {
        let mut value = fixture();
        value["opa_decision_log"]["input"]["action"] = json!("transfer_title");
        refresh_digest(&mut value);
        assert_eq!(evaluate(&value)["overall"], "DECISION_SCOPE_MISMATCH");
    }

    #[test]
    fn effect_must_bind_to_the_exact_operation_and_decision() {
        let mut value = fixture();
        value["effect_observation"]["operation_digest"] = json!("f".repeat(64));
        assert_eq!(evaluate(&value)["overall"], "EFFECT_BINDING_MISMATCH");

        let mut value = fixture();
        value["effect_observation"]["decision_id"] = json!("other-decision");
        assert_eq!(evaluate(&value)["overall"], "EFFECT_BINDING_MISMATCH");
    }

    #[test]
    fn common_cause_and_late_observation_do_not_reconcile() {
        let mut value = fixture();
        value["effect_observation"]["source_dependencies"] = json!(["provider_acknowledgement"]);
        assert_eq!(
            evaluate(&value)["overall"],
            "ACKNOWLEDGED_NOT_RECONCILED"
        );

        let mut value = fixture();
        value["effect_observation"]["observed_at"] = json!("2026-03-10T11:02:00Z");
        assert_eq!(evaluate(&value)["overall"], "INDETERMINATE");
    }

    #[test]
    fn partial_effect_is_not_promoted_to_full_success() {
        let mut value = fixture();
        value["effect_observation"]["outcome"] = json!("partially_applied");
        assert_eq!(
            evaluate(&value)["overall"],
            "PARTIAL_EFFECT_REQUIRES_RECONCILIATION"
        );
    }
}
