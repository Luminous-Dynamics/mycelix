//! Independent, effects-disabled checks for the life/civic reference contracts.
//!
//! This checker validates schema shape and corpus invariants. It does not determine
//! legal applicability, certify conformance, or authorize service actions.

use serde_json::{json, Value};
use sha1::{Digest, Sha1};
use std::collections::HashSet;
use std::fs;
use std::path::PathBuf;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn read_json(relative_path: &str) -> Value {
    let path = repo_root().join(relative_path);
    let bytes = fs::read(&path)
        .unwrap_or_else(|error| panic!("failed to read {}: {error}", path.display()));
    serde_json::from_slice(&bytes)
        .unwrap_or_else(|error| panic!("invalid JSON in {}: {error}", path.display()))
}

fn validator(relative_schema: &str) -> jsonschema::Validator {
    let schema = read_json(relative_schema);
    jsonschema::meta::validate(&schema)
        .unwrap_or_else(|error| panic!("invalid JSON Schema {relative_schema}: {error}"));
    jsonschema::options()
        .should_validate_formats(true)
        .build(&schema)
        .unwrap_or_else(|error| panic!("could not compile JSON Schema {relative_schema}: {error}"))
}


fn git_blob_sha1(relative_path: &str) -> String {
    let path = repo_root().join(relative_path);
    let bytes = fs::read(&path)
        .unwrap_or_else(|error| panic!("failed to hash {}: {error}", path.display()));
    let header = format!("blob {}\0", bytes.len());
    let mut hasher = Sha1::new();
    hasher.update(header.as_bytes());
    hasher.update(&bytes);
    format!("{:x}", hasher.finalize())
}

const INSTRUMENT_SCHEMA: &str = "schemas/life-civic/standard-instrument.schema.json";
const ASSESSMENT_SCHEMA: &str = "schemas/life-civic/applicability-assessment.schema.json";
const CORPUS_PATH: &str = "tests/reference/life-civic-standards/cases.json";

#[test]
fn corpus_manifest_points_to_existing_versioned_assets() {
    let manifest = read_json("tests/reference/life-civic-standards/manifest.json");
    let corpus = read_json(CORPUS_PATH);
    assert_eq!(manifest["corpus_id"], corpus["corpus_id"]);
    assert_eq!(manifest["version"], corpus["version"]);
    assert_eq!(manifest["cases_file"].as_str(), Some(CORPUS_PATH));

    let frozen_inputs = manifest["frozen_inputs"]
        .as_array()
        .expect("manifest frozen_inputs must be an array");
    assert_eq!(frozen_inputs.len(), 3, "all schema/corpus inputs must be pinned");
    for entry in frozen_inputs {
        let path = entry["path"]
            .as_str()
            .expect("frozen input path must be a string");
        let expected = entry["git_blob_sha1"]
            .as_str()
            .expect("frozen input Git blob SHA-1 must be a string");
        assert_eq!(expected.len(), 40, "{path} has malformed Git blob SHA");
        assert!(
            expected.bytes().all(|byte| byte.is_ascii_hexdigit()),
            "{path} has non-hexadecimal Git blob SHA"
        );
        assert_eq!(
            git_blob_sha1(path),
            expected,
            "frozen input changed without updating the reviewed manifest: {path}"
        );
    }

    let schema_paths = manifest["schemas"]
        .as_array()
        .expect("manifest schemas must be an array");
    assert_eq!(schema_paths.len(), 2);
    for path in schema_paths {
        let path = path.as_str().expect("schema path must be a string");
        let _ = read_json(path);
    }

    let snapshots = corpus["source_snapshots"]
        .as_array()
        .expect("source snapshots must be an array");
    assert!(snapshots.len() >= 4, "source lifecycle snapshots were lost");
    for snapshot in snapshots {
        assert!(
            snapshot["source_uri"]
                .as_str()
                .is_some_and(|uri| uri.starts_with("https://")),
            "source snapshot must retain a canonical HTTPS locator"
        );
    }
}

#[test]
fn both_schemas_are_valid_json_schema_documents() {
    let _instrument = validator(INSTRUMENT_SCHEMA);
    let _assessment = validator(ASSESSMENT_SCHEMA);
}

#[test]
fn working_draft_is_representable_without_claiming_publication() {
    let check = validator(INSTRUMENT_SCHEMA);
    let draft = json!({
        "instrument_id": "ISO:WD-37120",
        "title": "Sustainable cities and communities — Indicators for city services and quality of life",
        "artifact_kind": "standard",
        "issuer": {
            "name": "International Organization for Standardization",
            "role": "international_standards_body",
            "canonical_uri": "https://www.iso.org/"
        },
        "version": "Edition 3 working draft",
        "publication_status": "working_draft",
        "normative_status": "under_development",
        "formal_status_label": "Working draft; under development",
        "jurisdiction_scope": ["global"],
        "published_at": null,
        "effective_from": null,
        "effective_until": null,
        "supersedes": [],
        "source_evidence": [{
            "uri": "https://www.iso.org/standard/90316.html",
            "source_class": "official_metadata",
            "retrieved_at": "2026-10-14T10:00:00+02:00",
            "integrity_status": "transport_only",
            "notes": "Point-in-time catalogue metadata; not a published replacement edition."
        }],
        "license_access": {
            "metadata_reuse": "unknown",
            "full_text_retention": "unknown",
            "license_uri": null
        },
        "mapping_profile": {
            "profile_id": "life-civic-city-indicators-v0",
            "mapping_status": "partial",
            "known_gaps": ["No city-specific indicator mapping has been qualified."],
            "requires_domain_review": true
        },
        "checked_at": "2026-10-14T10:00:00+02:00",
        "currentness_horizon": null,
        "claim_ceiling": {
            "permitted_claims": ["The cited catalogue snapshot labels Edition 3 a working draft."],
            "prohibited_inferences": [
                "Edition 3 is a published International Standard.",
                "Any city conforms to the draft."
            ]
        }
    });
    assert!(check.is_valid(&draft), "valid draft metadata was rejected");

    let mut malformed_uri = draft.clone();
    malformed_uri["source_evidence"][0]["uri"] = json!("not-a-uri");
    assert!(
        !check.is_valid(&malformed_uri),
        "malformed source URI must be rejected when format checks are enabled"
    );

    let mut malformed_time = draft.clone();
    malformed_time["source_evidence"][0]["retrieved_at"] = json!("yesterday");
    assert!(
        !check.is_valid(&malformed_time),
        "malformed retrieval time must be rejected when format checks are enabled"
    );

    let mut mislabeled = draft;
    mislabeled["publication_status"] = json!("published_and_current");
    assert!(
        !check.is_valid(&mislabeled),
        "unknown lifecycle status must be rejected, not normalized silently"
    );
}

fn base_assessment(status: &str, review_status: &str) -> Value {
    json!({
        "assessment_id": "MYC-LCA-TEST",
        "instrument_ref": {
            "instrument_id": "EXAMPLE:INSTRUMENT",
            "version": "v1"
        },
        "context": {
            "use_case": "test applicability boundary",
            "jurisdiction": "ZA-GP",
            "evaluated_at": "2026-10-14T10:00:00+02:00",
            "facts_assumed": []
        },
        "result": {
            "status": status,
            "reason_codes": ["sufficient_scope_evidence"],
            "explanation": "Synthetic validator fixture only."
        },
        "evidence": [{
            "evidence_id": "E-1",
            "evidence_kind": "official_status_record",
            "source_uri": "https://example.org/status",
            "retrieved_at": "2026-10-14T10:00:00+02:00",
            "supports": "A schema-contract test fact.",
            "limitations": "Synthetic; not real legal evidence."
        }],
        "assumptions": [],
        "unresolved_questions": [],
        "human_review": {
            "required": true,
            "review_role": "qualified reviewer",
            "review_status": review_status
        },
        "assessed_at": "2026-10-14T10:00:00+02:00",
        "profile_version": "0.1.0",
        "claim_ceiling": {
            "permitted_claims": ["The fixture conforms to a local schema test."],
            "prohibited_inferences": ["The fixture establishes real-world legal applicability."]
        }
    })
}

#[test]
fn applicable_requires_evidence_and_completed_human_review() {
    let check = validator(ASSESSMENT_SCHEMA);

    let completed = base_assessment("applicable", "completed");
    assert!(
        check.is_valid(&completed),
        "an assessment with evidence and completed review should satisfy the contract"
    );

    let pending = base_assessment("applicable", "requested");
    assert!(
        !check.is_valid(&pending),
        "applicable status must not be accepted before human review is complete"
    );

    let mut no_evidence = base_assessment("applicable", "completed");
    no_evidence["evidence"] = json!([]);
    assert!(
        !check.is_valid(&no_evidence),
        "applicable status must not be accepted without supporting evidence"
    );

    let unknown = base_assessment("unknown", "not_started");
    assert!(
        check.is_valid(&unknown),
        "unknown must remain a representable, non-failing disposition"
    );
}

#[test]
fn corpus_ids_reasons_and_forbidden_claims_are_well_formed() {
    let corpus = read_json(CORPUS_PATH);
    assert_eq!(
        corpus["corpus_id"].as_str(),
        Some("MYC-LIFE-CIVIC-APPLICABILITY-001")
    );
    let cases = corpus["cases"]
        .as_array()
        .expect("corpus cases must be an array");
    assert!(
        cases.len() >= 8,
        "reference suite unexpectedly lost boundary cases"
    );

    let schema = read_json(ASSESSMENT_SCHEMA);
    let valid_statuses: HashSet<&str> =
        schema["properties"]["result"]["properties"]["status"]["enum"]
            .as_array()
            .expect("assessment schema must enumerate result statuses")
            .iter()
            .map(|value| value.as_str().expect("status enum must contain strings"))
            .collect();
    let valid_reasons: HashSet<&str> =
        schema["properties"]["result"]["properties"]["reason_codes"]["items"]["enum"]
            .as_array()
            .expect("assessment schema must enumerate reason codes")
            .iter()
            .map(|value| value.as_str().expect("reason enum must contain strings"))
            .collect();

    let mut ids = HashSet::new();
    for case in cases {
        let id = case["case_id"].as_str().expect("case_id must be a string");
        assert!(ids.insert(id), "duplicate case_id: {id}");
        let expected = &case["expected"];
        let status = expected["status"]
            .as_str()
            .expect("expected status must be a string");
        assert!(valid_statuses.contains(status), "{id} uses unknown status {status}");

        let reasons = expected["required_reason_codes"]
            .as_array()
            .expect("each case requires reason-code expectations");
        assert!(!reasons.is_empty(), "{id} has no expected reason code");
        for reason in reasons {
            let reason = reason.as_str().expect("reason code must be a string");
            assert!(
                valid_reasons.contains(reason),
                "{id} uses unknown reason code {reason}"
            );
        }

        let forbidden = expected["forbidden_claims"]
            .as_array()
            .expect("each case requires prohibited claims");
        assert!(!forbidden.is_empty(), "{id} has no prohibited-claim controls");
        assert!(case["synthetic"].is_boolean(), "{id} must say whether it is synthetic");
    }
}

#[test]
fn draft_registry_directory_and_transparency_negative_controls_remain_present() {
    let corpus = read_json(CORPUS_PATH);
    let cases = corpus["cases"].as_array().expect("cases array");
    let by_id: std::collections::HashMap<&str, &Value> = cases
        .iter()
        .map(|case| (case["case_id"].as_str().expect("case ID"), case))
        .collect();

    let draft = by_id.get("MYC-LCA-002").expect("ISO 37120 draft control missing");
    assert_eq!(draft["expected"]["status"], "indeterminate");
    assert_eq!(draft["expected"]["required_reason_codes"][0], "draft_not_published");

    let lifecycle = by_id.get("MYC-LCA-003").expect("ISO 37101 lifecycle control missing");
    assert!(
        lifecycle["expected"]["forbidden_claims"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| claim.as_str().unwrap().contains("automatically withdraws"))
    );

    let registry = by_id.get("MYC-LCA-007").expect("registry-authority control missing");
    assert!(
        registry["expected"]["forbidden_claims"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| claim.as_str().unwrap().contains("authorizes the action"))
    );

    let transparency = by_id.get("MYC-LCA-008").expect("transparency/truth control missing");
    assert!(
        transparency["expected"]["forbidden_claims"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| {
                claim
                    .as_str()
                    .unwrap()
                    .contains("proves the signed statement is true")
            })
    );
}
