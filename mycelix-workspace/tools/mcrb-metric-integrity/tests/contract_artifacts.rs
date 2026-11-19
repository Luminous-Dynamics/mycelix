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
        schema["properties"]["offer_ledger_policy"]["properties"]
            ["cross_host_requirement_source"],
        json_string("string")
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
        "idempotency_key",
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

    let evidence_required = receipt["properties"]["evidence"]["required"]
        .as_array()
        .unwrap();
    assert!(
        evidence_required
            .iter()
            .any(|value| value == "offer_ledger_artifact_digest_sha256")
    );
}

fn json_string(value: &str) -> Value {
    Value::String(value.to_string())
}
