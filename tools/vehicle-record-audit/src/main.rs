use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs;
use std::path::PathBuf;
use std::process::ExitCode;

const EXPECTED_SCHEMA_VERSION: &str = "vehicle-record-envelope/0.1";
const REQUIRED_FIXTURE_IDS: [&str; 19] = [
    "VD01_VALID_REPAIR_ATTESTATION",
    "VD02_UNSIGNED_REPAIR",
    "VD03_UNILATERAL_TRANSFER",
    "VD04_TWO_PARTY_TRANSFER",
    "VD05_COMPETING_TRANSFERS",
    "VD06_PUBLIC_PII",
    "VD07_PRIVATE_TRIP_LOCATION",
    "VD08_INCOMPATIBLE_PART",
    "VD09_PAYLOAD_DIGEST_MISMATCH",
    "VD10_WRONG_VEHICLE_GENERATION",
    "VD11_FAILED_POST_REPAIR_CHECK",
    "VD12_EXPIRED_WORKSHOP_CREDENTIAL",
    "VD13_RECALL_NOT_CLEARED",
    "VD14_APPEND_CORRECTION",
    "VD15_EXPORT_IMPORT",
    "VD16_DUPLICATE_EVENT_MUTATION",
    "VD17_UNSUPPORTED_TITLE_CLAIM",
    "VD18_EXPIRED_DISCLOSURE",
    "VD19_PUBLIC_RAW_VIN",
];

const REQUIRED_RECORD_INSTANCE_IDS: [&str; 9] = [
    "VRE01_VALID_MAINTENANCE_EVENT",
    "VRE02_MISSING_SIGNATURE",
    "VRE03_SIGNED_DIGEST_MISMATCH",
    "VRE04_MISSING_PROCEDURE_GRAPH_REF",
    "VRE05_OBSERVED_DURATION_WITHOUT_SOURCE",
    "VRE06_POSTCHECK_WITHOUT_EVIDENCE",
    "VRE07_SHARED_RAW_VIN",
    "VRE08_VERIFIED_WITHOUT_ATTESTATION",
    "VRE09_DISPUTED_WITHOUT_DISPUTE_REF",
];

type AuditResult<T> = Result<T, Vec<String>>;

#[derive(Debug, PartialEq, Eq)]
struct AuditSummary {
    fixture_count: usize,
    schema_sha256: String,
    fixture_sha256: String,
    record_instance_count: usize,
    record_instances_sha256: String,
}

fn default_schema_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../mycelix-commons/schemas/vehicle_record_envelope_v0_1.schema.json")
}

fn default_fixtures_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../mycelix-commons/fixtures/vehicle_dossier_fixtures_v0_1.json")
}

fn string_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn is_schema_valid(schema: &Value) -> bool {
    jsonschema::draft202012::meta::is_valid(schema)
}

fn instance_errors(schema: &Value, instance: &Value) -> Vec<String> {
    match jsonschema::draft202012::options()
        .should_validate_formats(true)
        .build(schema)
    {
        Ok(validator) => validator.iter_errors(instance).map(|error| error.to_string()).collect(),
        Err(error) => vec![format!("schema compilation failed: {error}")],
    }
}

fn check_envelope_schema(schema: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if string_at(schema, "$schema") != Some("https://json-schema.org/draft/2020-12/schema") {
        errors.push("schema must declare JSON Schema Draft 2020-12".to_owned());
    }
    if string_at(schema, "$id") != Some("urn:luminous-dynamics:mobility:vehicle-record-envelope:0.1") {
        errors.push("schema $id is not the v0.1 envelope identifier".to_owned());
    }
    if !is_schema_valid(schema) {
        errors.push("schema does not validate against the Draft 2020-12 meta-schema".to_owned());
    }
    if !schema
        .get("required")
        .and_then(Value::as_array)
        .is_some_and(|required| {
            required.iter().any(|entry| entry.as_str() == Some("signature"))
                && required.iter().any(|entry| entry.as_str() == Some("envelope_digest"))
        })
    {
        errors.push("signature and envelope_digest must remain required envelope fields".to_owned());
    }
    let signature_required: BTreeSet<&str> = schema
        .get("properties")
        .and_then(|properties| properties.get("signature"))
        .and_then(|signature| signature.get("required"))
        .and_then(Value::as_array)
        .map(|values| values.iter().filter_map(Value::as_str).collect())
        .unwrap_or_default();
    for field in ["signer_ref", "algorithm", "signature_b64", "signed_digest", "canonicalization_version", "signature_context"] {
        if !signature_required.contains(field) {
            errors.push(format!("signature.{field} must remain required"));
        }
    }
    if !schema
        .get("properties")
        .and_then(|properties| properties.get("vehicle_ref"))
        .and_then(|vehicle_ref| vehicle_ref.get("pattern"))
        .and_then(Value::as_str)
        .is_some_and(|pattern| pattern.starts_with("^veh_"))
    {
        errors.push("vehicle_ref must use the opaque veh_ reference form".to_owned());
    }
    if !schema
        .get("allOf")
        .and_then(Value::as_array)
        .is_some_and(|conditions| conditions.len() >= 3)
    {
        errors.push("privacy, independent-verification and dispute conditions must remain present".to_owned());
    }
    errors
}

fn check_fixture_corpus(fixtures: &Value) -> AuditResult<usize> {
    let mut errors = Vec::new();
    if string_at(fixtures, "corpus_version") != Some("mycelix-vehicle-dossier-fixtures/0.1") {
        errors.push("fixture corpus version mismatch".to_owned());
    }
    if string_at(fixtures, "status") != Some("synthetic-contract-fixtures-not-deployed") {
        errors.push("fixture corpus must preserve the not-deployed status".to_owned());
    }
    let Some(items) = fixtures.get("fixtures").and_then(Value::as_array) else {
        return Err(vec!["fixtures must be an array".to_owned()]);
    };
    if items.len() != REQUIRED_FIXTURE_IDS.len() {
        errors.push(format!(
            "expected {} fixtures, found {}",
            REQUIRED_FIXTURE_IDS.len(),
            items.len()
        ));
    }
    let mut seen = BTreeSet::new();
    for (index, item) in items.iter().enumerate() {
        let at = format!("fixtures[{index}]");
        let Some(id) = string_at(item, "id") else {
            errors.push(format!("{at}.id: missing or not a string"));
            continue;
        };
        if !seen.insert(id.to_owned()) {
            errors.push(format!("{at}: duplicate fixture id {id}"));
        }
        if !REQUIRED_FIXTURE_IDS.contains(&id) {
            errors.push(format!("{at}: unexpected fixture id {id}"));
        }
        if string_at(item, "expected").is_none_or(|value| value.trim().is_empty()) {
            errors.push(format!("{at}.expected: missing or blank"));
        }
        if !item.get("input").is_some_and(Value::is_object) {
            errors.push(format!("{at}.input: expected an object"));
        }
        let invariants = item.get("invariants").and_then(Value::as_array);
        if !invariants.is_some_and(|items| {
            !items.is_empty()
                && items.iter().all(|value| value.as_str().is_some_and(|text| !text.trim().is_empty()))
        }) {
            errors.push(format!("{at}.invariants: expected non-empty string array"));
        }
    }
    for id in REQUIRED_FIXTURE_IDS {
        if !seen.contains(id) {
            errors.push(format!("missing required fixture {id}"));
        }
    }
    let claims = fixtures.get("claim_ceiling").and_then(Value::as_array);
    if !claims.is_some_and(|items| {
        items.iter().filter_map(Value::as_str).any(|text| text.contains("do not establish legal title"))
            && items.iter().filter_map(Value::as_str).any(|text| text.contains("do not prove workmanship or safety"))
    }) {
        errors.push("fixture corpus claim ceiling was weakened".to_owned());
    }
    if errors.is_empty() { Ok(items.len()) } else { Err(errors) }
}

fn signature_binding_errors(record: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    let envelope_digest = record.get("envelope_digest").and_then(Value::as_str);
    let signed_digest = record.get("signature").and_then(|value| value.get("signed_digest")).and_then(Value::as_str);
    let payload_canonicalization = record
        .get("payload_digest")
        .and_then(|value| value.get("canonicalization_version"))
        .and_then(Value::as_str);
    let signature_canonicalization = record
        .get("signature")
        .and_then(|value| value.get("canonicalization_version"))
        .and_then(Value::as_str);
    let signature_context = record
        .get("signature")
        .and_then(|value| value.get("signature_context"))
        .and_then(Value::as_str);

    if envelope_digest.is_none() || signed_digest != envelope_digest {
        errors.push("signature.signed_digest must equal top-level envelope_digest".to_owned());
    }
    if payload_canonicalization.is_none() || signature_canonicalization != payload_canonicalization {
        errors.push("signature and payload digest canonicalization profiles must match".to_owned());
    }
    if signature_context != Some("mycelix-vehicle-record-envelope/0.1") {
        errors.push("signature context domain separator mismatch".to_owned());
    }
    errors
}


fn validate_record_instance_fixtures(
    schema: &Value,
    document: &Value,
    raw_bytes: &[u8],
) -> AuditResult<(usize, String)> {
    let mut errors = Vec::new();
    if string_at(document, "corpus_version") != Some("vehicle-record-envelope-instances/0.1") {
        errors.push("record instances: unexpected corpus version".to_owned());
    }
    if string_at(document, "status") != Some("synthetic-record-instances-not-production-signatures") {
        errors.push("record instances: synthetic signature status must remain explicit".to_owned());
    }
    let Some(fixtures) = document.get("fixtures").and_then(Value::as_array) else {
        return Err(vec!["record instances.fixtures must be an array".to_owned()]);
    };
    if fixtures.len() != REQUIRED_RECORD_INSTANCE_IDS.len() {
        errors.push(format!(
            "record instances: expected {} fixtures, found {}",
            REQUIRED_RECORD_INSTANCE_IDS.len(), fixtures.len()
        ));
    }

    let validator = match jsonschema::draft202012::options()
        .should_validate_formats(true)
        .build(schema)
    {
        Ok(validator) => Some(validator),
        Err(error) => {
            errors.push(format!("record instances: schema compilation failed: {error}"));
            None
        }
    };
    let mut seen = BTreeSet::new();
    for (index, fixture) in fixtures.iter().enumerate() {
        let at = format!("record instances[{index}]");
        let Some(id) = string_at(fixture, "id") else {
            errors.push(format!("{at}: missing fixture ID"));
            continue;
        };
        if !seen.insert(id.to_owned()) {
            errors.push(format!("{at}: duplicate fixture ID {id}"));
        }
        if !REQUIRED_RECORD_INSTANCE_IDS.contains(&id) {
            errors.push(format!("{at}: unregistered fixture ID {id}"));
        }
        let expectation = string_at(fixture, "expectation").unwrap_or("");
        let Some(record) = fixture.get("record") else {
            errors.push(format!("{at}: missing record instance"));
            continue;
        };
        let Some(validator) = validator.as_ref() else { continue; };
        let schema_errors: Vec<String> = validator.iter_errors(record).map(|error| error.to_string()).collect();
        let signature_errors = signature_binding_errors(record);
        match expectation {
            "accept" => {
                if !schema_errors.is_empty() {
                    errors.push(format!("{id}: valid instance failed schema: {}", schema_errors.join("; ")));
                }
                if !signature_errors.is_empty() {
                    errors.push(format!("{id}: valid instance failed digest/context binding: {}", signature_errors.join("; ")));
                }
            }
            "reject_schema" => {
                if schema_errors.is_empty() {
                    errors.push(format!("{id}: invalid instance unexpectedly passed schema"));
                }
            }
            "reject_signature_binding" => {
                if !schema_errors.is_empty() {
                    errors.push(format!("{id}: digest-binding negative control also failed schema"));
                } else if signature_errors.is_empty() {
                    errors.push(format!("{id}: digest-binding negative control unexpectedly passed"));
                }
            }
            _ => errors.push(format!("{id}: unknown expectation {expectation}")),
        }
    }
    for id in REQUIRED_RECORD_INSTANCE_IDS {
        if !seen.contains(id) {
            errors.push(format!("record instances: missing fixture {id}"));
        }
    }
    let ceiling = document.get("claim_ceiling").and_then(Value::as_array);
    if !ceiling.is_some_and(|items| {
        items.iter().filter_map(Value::as_str).any(|text| text.contains("does not cryptographically verify them"))
            && items.iter().filter_map(Value::as_str).any(|text| text.contains("No fixture establishes legal title"))
    }) {
        errors.push("record instances: explicit synthetic/claim ceiling required".to_owned());
    }
    if errors.is_empty() {
        Ok((fixtures.len(), format!("{:x}", Sha256::digest(raw_bytes))))
    } else {
        Err(errors)
    }
}

fn default_record_instances_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../mycelix-commons/fixtures/vehicle_record_envelopes_v0_1.json")
}

fn valid_repair_record() -> Value {
    json!({
        "schema_version": EXPECTED_SCHEMA_VERSION,
        "event_id": "evt_repair_20261017_01",
        "vehicle_ref": "veh_0123456789abcdef",
        "event_kind": "maintenance_performed",
        "actor_ref": "actor_workshop_01",
        "occurred_at": "2026-10-17T10:00:00Z",
        "recorded_at": "2026-10-17T10:01:00Z",
        "payload_digest": {
            "algorithm": "sha-256",
            "value": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "canonicalization_version": "json-c14n-v1"
        },
        "envelope_digest": "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
        "evidence_refs": [{
            "artifact_kind": "repair_order",
            "artifact_ref": "repair-order-2026-001",
            "artifact_digest": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
            "scope_note": "Synthetic repair-order reference"
        }],
        "visibility_policy": "shared_pseudonymous",
        "claim_status": "counterparty_attested",
        "signature": {
            "signer_ref": "actor_workshop_01",
            "algorithm": "ed25519",
            "signature_b64": "QUJDREVGR0hJSktMTU5PUFFSU1RVVldYWVo=",
            "signed_digest": "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
            "canonicalization_version": "json-c14n-v1",
            "signature_context": "mycelix-vehicle-record-envelope/0.1",
            "credential_ref": "credential_workshop_01"
        },
        "privacy_review": {
            "contains_direct_identifiers": false,
            "consent_ref": "consent_repair_2026_001",
            "review_version": "privacy-review-v0.1"
        },
        "payload": {
            "work_type": "scheduled_service",
            "repair_task_graph_ref": "art_1123456789abcdef",
            "repair_task_graph_digest": "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff",
            "procedure_revision": "procedure-v1",
            "work_status": "performed",
            "task_results": [{
                "task_id": "task_service01",
                "status": "performed",
                "component_ref": "part_serviceitem01",
                "component_revision": "A",
                "duration": {
                    "value": 8,
                    "unit": "min",
                    "basis": "observed",
                    "observation_ref": "observation_service_001",
                    "observed_at": "2026-10-17T10:01:00Z",
                    "technician_ref": "actor_workshop_01",
                    "tool_context": "Synthetic fixture timing only.",
                    "source_refs": ["work-order-2026-001"]
                },
                "evidence_refs": ["work-order-2026-001"]
            }],
            "post_repair_verification": {
                "result": "passed",
                "observation_ref": "measurement-001",
                "evidence_refs": ["measurement-001"]
            }
        }
    })
}

fn main() -> ExitCode {
    let mut schema_path = default_schema_path();
    let mut fixtures_path = default_fixtures_path();
    let mut args = env::args_os().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--schema" {
            let Some(value) = args.next() else {
                eprintln!("SCHEMA_AUDIT_FAIL: --schema requires a path");
                return ExitCode::from(2);
            };
            schema_path = PathBuf::from(value);
        } else if arg == "--fixtures" {
            let Some(value) = args.next() else {
                eprintln!("SCHEMA_AUDIT_FAIL: --fixtures requires a path");
                return ExitCode::from(2);
            };
            fixtures_path = PathBuf::from(value);
        } else if arg == "--help" || arg == "-h" {
            println!("Usage: mycelix-vehicle-record-audit [--schema PATH] [--fixtures PATH]");
            println!("Checks schema meta-validity and synthetic fixture structure; does not verify signatures or legal title.");
            return ExitCode::SUCCESS;
        } else {
            eprintln!("SCHEMA_AUDIT_FAIL: unknown argument {}", arg.to_string_lossy());
            return ExitCode::from(2);
        }
    }

    let record_instances_path = default_record_instances_path();
    let record_instances_bytes = match fs::read(&record_instances_path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("SCHEMA_AUDIT_FAIL: cannot read {}: {error}", record_instances_path.display());
            return ExitCode::from(2);
        }
    };
    let record_instances: Value = match serde_json::from_slice(&record_instances_bytes) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("SCHEMA_AUDIT_FAIL: record instance JSON parse error: {error}");
            return ExitCode::from(2);
        }
    };

    let schema_bytes = match fs::read(&schema_path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("SCHEMA_AUDIT_FAIL: cannot read {}: {error}", schema_path.display());
            return ExitCode::from(2);
        }
    };
    let fixture_bytes = match fs::read(&fixtures_path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("SCHEMA_AUDIT_FAIL: cannot read {}: {error}", fixtures_path.display());
            return ExitCode::from(2);
        }
    };
    let schema: Value = match serde_json::from_slice(&schema_bytes) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("SCHEMA_AUDIT_FAIL: schema JSON parse error: {error}");
            return ExitCode::from(2);
        }
    };
    let fixtures: Value = match serde_json::from_slice(&fixture_bytes) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("SCHEMA_AUDIT_FAIL: fixture JSON parse error: {error}");
            return ExitCode::from(2);
        }
    };

    let mut errors = check_envelope_schema(&schema);
    let fixture_count = match check_fixture_corpus(&fixtures) {
        Ok(count) => Some(count),
        Err(fixture_errors) => {
            errors.extend(fixture_errors);
            None
        }
    };
    if errors.is_empty() {
        let record = valid_repair_record();
        errors.extend(instance_errors(&schema, &record));
        errors.extend(signature_binding_errors(&record));
    }
    let record_instance_result = match validate_record_instance_fixtures(&schema, &record_instances, &record_instances_bytes) {
        Ok(result) => Some(result),
        Err(instance_errors) => {
            errors.extend(instance_errors);
            None
        }
    };
    if errors.is_empty() {
        let fixture_count = fixture_count.unwrap_or_default();
        let (record_instance_count, record_instances_sha256) =
            record_instance_result.expect("record instance fixtures valid when audit succeeds");
        let summary = AuditSummary {
            fixture_count,
            schema_sha256: format!("{:x}", Sha256::digest(&schema_bytes)),
            fixture_sha256: format!("{:x}", Sha256::digest(&fixture_bytes)),
            record_instance_count,
            record_instances_sha256,
        };
        println!(
            "SCHEMA_AUDIT_PASS schema_draft=2020-12 decision_fixtures={} schema_sha256={} decision_fixtures_sha256={} record_instances={} record_instances_sha256={}",
            summary.fixture_count, summary.schema_sha256, summary.fixture_sha256,
            summary.record_instance_count, summary.record_instances_sha256
        );
        println!("CLAIM_CEILING schema/corpus conformance and declared digest-binding consistency only; no cryptographic verification, truth, legal title or safety result");
        ExitCode::SUCCESS
    } else {
        eprintln!("SCHEMA_AUDIT_FAIL errors={}", errors.len());
        for error in errors {
            eprintln!("- {error}");
        }
        ExitCode::from(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn schema() -> Value {
        serde_json::from_str(include_str!("../../../mycelix-commons/schemas/vehicle_record_envelope_v0_1.schema.json"))
            .expect("schema should be valid JSON")
    }

    fn fixtures() -> Value {
        serde_json::from_str(include_str!("../../../mycelix-commons/fixtures/vehicle_dossier_fixtures_v0_1.json"))
            .expect("fixtures should be valid JSON")
    }

    fn record_instances() -> Value {
        serde_json::from_str(include_str!("../../../mycelix-commons/fixtures/vehicle_record_envelopes_v0_1.json"))
            .expect("record instance corpus should be JSON")
    }

    fn passes(schema: &Value, record: &Value) -> bool {
        instance_errors(schema, record).is_empty()
    }

    #[test]
    fn schema_is_valid_draft_2020_12() {
        let schema = schema();
        assert!(is_schema_valid(&schema));
        assert!(check_envelope_schema(&schema).is_empty());
    }

    #[test]
    fn frozen_fixture_census_is_complete() {
        let fixtures = fixtures();
        assert_eq!(check_fixture_corpus(&fixtures), Ok(19));
    }

    #[test]
    fn frozen_record_instances_validate_all_schema_and_binding_expectations() {
        let schema=schema();
        let doc=record_instances();
        let bytes=serde_json::to_vec(&doc).expect("fixture bytes");
        let result=validate_record_instance_fixtures(&schema,&doc,&bytes).expect("record instance corpus");
        assert_eq!(result.0,9);
        assert_eq!(result.1.len(),64);
    }

    #[test]
    fn digest_mismatch_negative_control_cannot_be_relabelled_valid() {
        let mut doc=record_instances();
        let fixtures=doc["fixtures"].as_array_mut().expect("fixture array");
        let entry=fixtures.iter_mut().find(|x|x["id"]=="VRE03_SIGNED_DIGEST_MISMATCH").expect("digest fixture");
        entry["expectation"]=Value::String("accept".to_owned());
        let schema=schema();
        let bytes=serde_json::to_vec(&doc).expect("fixture bytes");
        assert!(validate_record_instance_fixtures(&schema,&doc,&bytes).is_err());
    }

    #[test]
    fn minimum_repair_record_has_valid_schema_and_digest_binding() {
        let schema = schema();
        let record = valid_repair_record();
        assert!(passes(&schema, &record));
        assert!(signature_binding_errors(&record).is_empty());
    }

    #[test]
    fn maintenance_event_requires_repair_graph_and_task_lineage() {
        let schema=schema();
        let mut record=valid_repair_record();
        record["payload"].as_object_mut().expect("payload object").remove("repair_task_graph_ref");
        assert!(!passes(&schema,&record));
    }

    #[test]
    fn observed_task_duration_requires_observation_source() {
        let schema=schema();
        let mut record=valid_repair_record();
        record["payload"]["task_results"][0]["duration"].as_object_mut().expect("duration").remove("observation_ref");
        assert!(!passes(&schema,&record));
    }

    #[test]
    fn post_repair_pass_requires_a_verification_receipt() {
        let schema=schema();
        let mut record=valid_repair_record();
        record["payload"]["post_repair_verification"].as_object_mut().expect("post-check").remove("evidence_refs");
        assert!(!passes(&schema,&record));
    }

    #[test]
    fn signature_digest_must_bind_the_envelope_digest() {
        let mut record = valid_repair_record();
        record["signature"]["signed_digest"] =
            Value::String("dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd".to_owned());
        assert!(passes(&schema(), &record));
        assert!(!signature_binding_errors(&record).is_empty());
    }

    #[test]
    fn signature_domain_and_canonicalization_must_match() {
        let mut record = valid_repair_record();
        record["signature"]["signature_context"] = Value::String("other-contract/1".to_owned());
        assert!(!signature_binding_errors(&record).is_empty());
        record = valid_repair_record();
        record["signature"]["canonicalization_version"] = Value::String("other-c14n".to_owned());
        assert!(!signature_binding_errors(&record).is_empty());
    }

    #[test]
    fn signature_is_mandatory() {
        let schema = schema();
        let mut record = valid_repair_record();
        record.as_object_mut().expect("object").remove("signature");
        assert!(!passes(&schema, &record));
    }

    #[test]
    fn public_payload_rejects_nested_owner_identifiers() {
        let schema = schema();
        let mut record = valid_repair_record();
        record["payload"]["nested"] = json!({"private":{"owner_name":"Example Person"}});
        assert!(!passes(&schema, &record));
    }

    #[test]
    fn shared_record_rejects_direct_identifier_flag() {
        let schema = schema();
        let mut record = valid_repair_record();
        record["privacy_review"]["contains_direct_identifiers"] = Value::Bool(true);
        assert!(!passes(&schema, &record));
    }

    #[test]
    fn independently_verified_requires_scoped_attestation() {
        let schema = schema();
        let mut record = valid_repair_record();
        record["claim_status"] = Value::String("independently_verified".to_owned());
        assert!(!passes(&schema, &record));
        record["verification_attestations"] = json!([{
            "verification_id":"verification_0001",
            "verifier_ref":"verifier_0001",
            "method":"independent_document_review",
            "scope":"Repair order and post-repair check only",
            "outcome":"verified",
            "verified_at":"2026-10-17T10:02:00Z",
            "credential_ref":"credential_verifier_01",
            "evidence_refs":["repair-order-2026-001","measurement-001"]
        }]);
        assert!(passes(&schema, &record));
    }

    #[test]
    fn disputed_record_requires_a_dispute_reference() {
        let schema = schema();
        let mut record = valid_repair_record();
        record["claim_status"] = Value::String("disputed".to_owned());
        assert!(!passes(&schema, &record));
        record["dispute_refs"] = json!(["dispute_000001"]);
        assert!(passes(&schema, &record));
    }

    #[test]
    fn malformed_date_time_is_rejected_when_format_checks_are_enabled() {
        let schema = schema();
        let mut record = valid_repair_record();
        record["occurred_at"] = Value::String("yesterday".to_owned());
        assert!(!passes(&schema, &record));
    }

    #[test]
    fn unknown_top_level_fields_are_rejected() {
        let schema = schema();
        let mut record = valid_repair_record();
        record["owner_name"] = Value::String("Example Person".to_owned());
        assert!(!passes(&schema, &record));
    }

    #[test]
    fn raw_vin_payload_key_is_rejected_in_shared_record() {
        let schema = schema();
        let mut record = valid_repair_record();
        record["payload"]["raw_vin"] = Value::String("SYNTHETICVIN0000001".to_owned());
        assert!(!passes(&schema, &record));
    }

    #[test]
    fn invalid_vehicle_reference_is_rejected() {
        let schema = schema();
        let mut record = valid_repair_record();
        record["vehicle_ref"] = Value::String("VIN12345".to_owned());
        assert!(!passes(&schema, &record));
    }
}
