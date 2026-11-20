use serde_json::Value;
use std::collections::BTreeSet;
use std::fs;
use std::path::Path;

fn repository_root() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .expect("crate path must be nested under mycelix-workspace/tools")
}

fn load_json(relative_path: &str) -> Value {
    let path = repository_root().join(relative_path);
    let text = fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("failed to read {}: {error}", path.display()));
    serde_json::from_str(&text)
        .unwrap_or_else(|error| panic!("invalid JSON in {}: {error}", path.display()))
}

#[test]
fn scenario_manifest_and_schema_agree_on_offer_ledger_policy() {
    let manifest = load_json("docs/benchmarks/MCRB_V0_1_SCENARIO_MANIFEST.json");
    let schema = load_json("docs/benchmarks/MCRB_V0_1_SCENARIO_MANIFEST.schema.json");

    assert_eq!(
        manifest["offer_ledger_policy"]["schema_version"],
        "mycelix-mcrb-offer-ledger-v0.1"
    );
    assert!(
        schema["required"]
            .as_array()
            .unwrap()
            .iter()
            .any(|value| value == "offer_ledger_policy")
    );
    assert_eq!(
        manifest["offer_ledger_policy"]["operation_contract_registry"]["schema_version"],
        "mycelix-mcrb-operation-contract-registry-v0.1"
    );
    assert_eq!(
        manifest["offer_ledger_policy"]["operation_contract_registry"]["registry_path"],
        "docs/benchmarks/MCRB_OPERATION_CONTRACTS_V0_1.json"
    );
    assert!(
        schema["properties"]["offer_ledger_policy"]["required"]
            .as_array()
            .unwrap()
            .iter()
            .any(|value| value == "operation_contract_registry")
    );
    assert_eq!(
        schema["properties"]["offer_ledger_policy"]["properties"]
            ["cross_host_requirement_source"]["type"],
        "string"
    );

    let scenarios = manifest["scenarios"].as_array().unwrap();
    let mut ids = BTreeSet::new();
    for scenario in scenarios {
        let id = scenario["id"].as_str().expect("scenario id");
        assert!(ids.insert(id.to_string()), "duplicate scenario id: {id}");
        assert!(
            scenario["qops_requires_cross_host_visibility"].is_boolean(),
            "{id} must freeze the remote-visibility requirement"
        );
    }
    assert!(
        scenarios.iter().any(|scenario| scenario["id"] == "MCRB-23"),
        "offer-ledger integrity scenario must remain in the manifest"
    );
}

#[test]
fn offer_ledger_schema_binds_the_fields_consumed_by_the_rust_verifier() {
    let schema = load_json("docs/benchmarks/MCRB_OFFER_LEDGER_V0_1.schema.json");
    let required = schema["required"].as_array().unwrap();
    for field in [
        "run_id",
        "scenario_id",
        "requires_cross_host_visibility",
        "scenario_contract_digest_sha256",
        "operation_contract_registry_digest_sha256",
        "workload_manifest_digest_sha256",
        "generator_digest_sha256",
        "parameter_set_digest_sha256",
        "measurement_start_ticks",
        "measurement_end_ticks",
        "ticks_per_second",
        "entry_count",
        "entries",
        "ledger_digest_sha256",
    ] {
        assert!(
            required.iter().any(|value| value == field),
            "offer-ledger schema missing required field {field}"
        );
    }

    let entry_required = schema["properties"]["entries"]["items"]["required"]
        .as_array()
        .unwrap();
    for field in [
        "sequence",
        "operation_id",
        "request_digest_sha256",
        "idempotency_key_digest_sha256",
        "operation_contract_id",
        "operation_contract_version",
        "operation_contract_digest_sha256",
        "scheduled_monotonic_tick",
        "requires_idempotency_key",
        "monotonic_clock_id",
        "monotonic_clock_epoch",
    ] {
        assert!(
            entry_required.iter().any(|value| value == field),
            "offer-ledger entry schema missing required field {field}"
        );
    }
}

#[test]
fn run_receipt_binds_the_verified_ledger_and_scenario_visibility_policy() {
    let receipt = load_json("docs/benchmarks/MCRB_V0_1_RUN_RECEIPT.schema.json");
    let qops = &receipt["properties"]["qops"];
    let offer = &qops["properties"]["offer_ledger"];
    let required = offer["required"].as_array().unwrap();

    for field in [
        "schema_version",
        "run_id",
        "scenario_id",
        "requires_cross_host_visibility",
        "scenario_contract_digest_sha256",
        "operation_contract_registry_digest_sha256",
        "digest_sha256",
        "entry_count",
        "generator_digest_sha256",
        "workload_manifest_digest_sha256",
        "parameter_set_digest_sha256",
        "artifact_digest_sha256",
    ] {
        assert!(
            required.iter().any(|value| value == field),
            "QOPS receipt missing offer-ledger field {field}"
        );
    }

    let qops_required = qops["required"].as_array().unwrap();
    assert!(
        qops_required.iter().any(|value| value == "correctness_gate_set"),
        "QOPS receipt must require an independently verified correctness gate set"
    );
    assert!(
        qops["properties"].get("hard_correctness_gates_passed").is_none(),
        "a caller-supplied boolean must not qualify QOPS"
    );

    let gate_set = &qops["properties"]["correctness_gate_set"];
    let gate_required = gate_set["required"].as_array().unwrap();
    for field in [
        "status",
        "scenario_contract_digest_sha256",
        "trust_policy_digest_sha256",
        "required_gate_ids",
        "gate_results",
        "attestation_bundle_digest_sha256",
        "verifier_artifact_digest_sha256",
        "slsa_build_provenance_digests",
    ] {
        assert!(
            gate_required.iter().any(|value| value == field),
            "correctness gate set missing field {field}"
        );
    }

    let evidence_required = receipt["properties"]["evidence"]["required"]
        .as_array()
        .unwrap();
    for field in [
        "offer_ledger_artifact_digest_sha256",
        "in_toto_bundle_digest_sha256",
        "attestation_trust_policy_digest_sha256",
        "slsa_build_provenance_digests",
    ] {
        assert!(
            evidence_required.iter().any(|value| value == field),
            "run receipt evidence missing field {field}"
        );
    }
}

#[test]
fn correctness_gate_predicate_schema_binds_run_source_runtime_and_trust_policy() {
    let schema = load_json("docs/benchmarks/MCRB_GATE_ATTESTATION_V0_1.schema.json");
    let required = schema["required"].as_array().unwrap();

    for field in [
        "run_id",
        "scenario_id",
        "scenario_contract_digest_sha256",
        "source",
        "runtime",
        "topology",
        "gate",
        "evidence",
        "verifier",
        "trust_policy",
    ] {
        assert!(
            required.iter().any(|value| value == field),
            "gate predicate schema missing field {field}"
        );
    }

    assert_eq!(
        schema["properties"]["gate"]["properties"]["result"]["enum"][0],
        "pass"
    );
    assert_eq!(
        schema["properties"]["gate"]["properties"]["result"]["enum"][1],
        "fail"
    );
    assert_eq!(
        schema["properties"]["gate"]["properties"]["result"]["enum"][2],
        "incomplete"
    );
}



#[test]
fn dsse_envelope_uses_the_standard_payload_type_wire_name() {
    use mycelix_mcrb_metric_integrity::attestation::{DsseEnvelope, DsseSignature};

    let envelope = DsseEnvelope {
        payload_type: "application/vnd.in-toto+json".to_string(),
        payload: "e30=".to_string(),
        signatures: vec![DsseSignature {
            keyid: "test-key".to_string(),
            sig: "AA==".to_string(),
        }],
    };
    let value = serde_json::to_value(&envelope).expect("DSSE envelope must serialize");

    assert_eq!(value["payloadType"], "application/vnd.in-toto+json");
    assert!(value.get("payload_type").is_none());

    let interoperable_json = serde_json::json!({
        "payloadType": "application/vnd.in-toto+json",
        "payload": "e30=",
        "signatures": [{"keyid": "test-key", "sig": "AA=="}]
    });
    let decoded: DsseEnvelope = serde_json::from_value(interoperable_json)
        .expect("standard DSSE payloadType envelope must deserialize");
    assert_eq!(decoded.payload_type, "application/vnd.in-toto+json");
}

#[test]
fn dsse_schema_requires_the_standard_payload_type_wire_name() {
    let schema = load_json("docs/benchmarks/MCRB_DSSE_ENVELOPE_V0_1.schema.json");
    let required = schema["required"].as_array().unwrap();
    assert!(required.iter().any(|value| value == "payloadType"));
    assert!(!required.iter().any(|value| value == "payload_type"));
    assert!(schema["properties"].get("payloadType").is_some());
    assert!(schema["properties"].get("payload_type").is_none());
}


#[test]
fn outcome_attestation_schemas_preserve_evidence_and_dependency_dimensions() {
    let ledger = load_json("docs/benchmarks/MCRB_OUTCOME_LEDGER_V0_1.schema.json");
    let predicate = load_json("docs/benchmarks/MCRB_OUTCOME_ATTESTATION_V0_1.schema.json");
    let trust = load_json("docs/benchmarks/MCRB_OUTCOME_TRUST_POLICY_V0_1.schema.json");

    for field in [
        "offer_ledger_digest_sha256",
        "operation_id",
        "request_digest_sha256",
        "operation_contract_registry_digest_sha256",
        "operation_contract_id",
        "operation_contract_version",
        "operation_contract_digest_sha256",
        "attempt_state",
        "effect_state",
        "outcome_class",
        "attestations",
    ] {
        assert!(
            ledger["properties"]["entries"]["items"]["required"]
                .as_array()
                .unwrap()
                .iter()
                .any(|value| value == field),
            "outcome ledger entry missing {field}"
        );
    }

    assert_eq!(
        predicate["properties"]["predicateType"],
        "https://luminousdynamics.io/attestations/mcrb-operation-outcome/v0.1"
    );
    let predicate_required = predicate["properties"]["predicate"]["required"]
        .as_array()
        .unwrap();
    for field in [
        "offer_sequence",
        "request_digest_sha256",
        "idempotency_key_digest_sha256",
        "operation_contract_registry_digest_sha256",
        "operation_contract_id",
        "operation_contract_version",
        "operation_contract_digest_sha256",
        "attempt_state",
        "effect_state",
        "outcome_class",
        "source",
        "evidence",
        "trust_policy_digest_sha256",
    ] {
        assert!(predicate_required.iter().any(|value| value == field));
    }

    let trust_required = trust["required"].as_array().unwrap();
    for field in [
        "effect_dependency_ids",
        "effect_dependency_closure_complete",
        "sources",
    ] {
        assert!(trust_required.iter().any(|value| value == field));
    }
    assert!(
        trust["properties"]["sources"]["items"]["required"]
            .as_array()
            .unwrap()
            .iter()
            .any(|value| value == "dependency_closure_complete")
    );
}

#[test]
fn qops_receipt_keeps_outcome_verification_separate_from_trust_policy() {
    let receipt = load_json("docs/benchmarks/MCRB_V0_1_RUN_RECEIPT.schema.json");
    let qops = &receipt["properties"]["qops"];
    let required = qops["required"].as_array().unwrap();
    assert!(required.iter().any(|value| value == "outcome_ledger"));
    assert!(required.iter().any(|value| value == "outcome_trust_policy"));

    let ledger_required = qops["properties"]["outcome_ledger"]["required"]
        .as_array()
        .unwrap();
    assert!(ledger_required.iter().any(|value| value == "verification_status"));
    assert!(ledger_required.iter().any(|value| value == "ledger_digest_sha256"));

    let policy_required = qops["properties"]["outcome_trust_policy"]["required"]
        .as_array()
        .unwrap();
    assert!(policy_required.iter().any(|value| value == "policy_digest_sha256"));
    assert!(policy_required.iter().any(|value| value == "dependency_closure_complete"));
}

#[test]
fn qops_receipt_labels_caller_outcomes_as_diagnostic_only() {
    let receipt = load_json("docs/benchmarks/MCRB_V0_1_RUN_RECEIPT.schema.json");
    let qops = &receipt["properties"]["qops"];
    assert!(
        qops["required"].as_array().unwrap().iter().any(|value| value == "asserted_outcome_samples")
    );
    assert!(
        qops["properties"]["asserted_outcome_samples"]["description"]
            .as_str().unwrap().contains("Diagnostic only")
    );

    for field in [
        "classified_outcomes",
        "qualified_successes",
        "correct_denials",
        "unexpected_denials",
        "known_failures",
        "unknown_outcomes",
        "duplicate_external_effects",
    ] {
        assert!(
            qops["properties"][field]["description"]
                .as_str().unwrap().contains("Verified outcome-ledger count only"),
            "{field} must describe verified counts only"
        );
    }

    let rules = receipt["allOf"].as_array().unwrap();
    assert!(rules.iter().any(|rule| {
        rule["if"]["properties"]["qops"]["properties"]["status"] == "eligible"
            && rule["then"]["properties"]["qops"]["properties"]["outcome_evidence_status"] == "verified"
    }));
}
