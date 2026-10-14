//! Independent, effects-disabled checks for the life/civic reference contracts.
//!
//! This checker validates schema shape and corpus invariants. It does not determine
//! legal applicability, certify conformance, or authorize service actions.

use serde_json::{json, Value};
use sha1::{Digest, Sha1};
use std::collections::{HashMap, HashSet};
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

fn find_service_fixture<'a>(fixtures: &'a [Value], id: &str) -> &'a Value {
    fixtures
        .iter()
        .find(|fixture| fixture["fixture_id"] == id)
        .unwrap_or_else(|| panic!("missing required service fixture {id}"))
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

fn validate_service_claim_semantics(instance: &Value) -> Result<(), String> {
    let evidence = instance["evidence"]
        .as_array()
        .ok_or_else(|| "evidence must be an array".to_string())?;
    let mut records: HashMap<String, (String, String, String, String)> = HashMap::new();

    for item in evidence {
        let id = item["evidence_id"]
            .as_str()
            .ok_or_else(|| "evidence record has no string evidence_id".to_string())?;
        let kind = item["kind"]
            .as_str()
            .ok_or_else(|| format!("{id} has no string kind"))?;
        let independence = item["independence"]
            .as_str()
            .ok_or_else(|| format!("{id} has no independence classification"))?;
        let source_uri = item["source_uri"]
            .as_str()
            .ok_or_else(|| format!("{id} has no source URI"))?;
        let retrieval_state = item["retrieval_state"]
            .as_str()
            .ok_or_else(|| format!("{id} has no retrieval_state"))?;

        match retrieval_state {
            "captured"
                if item["retrieved_at"].as_str().is_some()
                    && item["checked_at"].as_str().is_some() => {}
            "synthetic_fixture"
                if item["retrieved_at"].is_null() && item["checked_at"].is_null() => {}
            "unavailable"
                if item["retrieved_at"].is_null() && item["checked_at"].as_str().is_some() => {}
            "captured" => {
                return Err(format!(
                    "{id} requires both retrieved_at and checked_at timestamps"
                ));
            }
            "synthetic_fixture" => {
                return Err(format!(
                    "{id} synthetic fixture must not claim retrieval or check timestamps"
                ));
            }
            "unavailable" => {
                return Err(format!(
                    "{id} unavailable source requires checked_at and null retrieved_at"
                ));
            }
            _ => return Err(format!("{id} has an unknown retrieval_state")),
        }

        if records
            .insert(
                id.to_string(),
                (
                    kind.to_string(),
                    independence.to_string(),
                    source_uri.to_string(),
                    retrieval_state.to_string(),
                ),
            )
            .is_some()
        {
            return Err(format!("duplicate evidence_id: {id}"));
        }
    }

    let outcome = &instance["outcome"];
    let status = outcome["status"]
        .as_str()
        .ok_or_else(|| "outcome has no string status".to_string())?;
    let verification_ids = outcome["verification_evidence_ids"]
        .as_array()
        .ok_or_else(|| "verification_evidence_ids must be an array".to_string())?;
    let verified_at = &outcome["verified_at"];

    if status == "independently_verified_complete" {
        if instance["review_required"] != true {
            return Err("independent verification requires review_required=true".to_string());
        }
        if instance["review_status"] != "completed" {
            return Err("independent verification requires completed review".to_string());
        }
        if verification_ids.is_empty() {
            return Err("independent verification requires evidence IDs".to_string());
        }
        if verified_at.as_str().is_none() {
            return Err("independent verification requires a timestamp".to_string());
        }
        let mut unique_ids = HashSet::new();
        for value in verification_ids {
            let id = value
                .as_str()
                .ok_or_else(|| "verification evidence ID must be a string".to_string())?;
            if !unique_ids.insert(id) {
                return Err(format!("duplicate verification evidence ID: {id}"));
            }
            let (kind, independence, _source_uri, retrieval_state) = records
                .get(id)
                .ok_or_else(|| format!("verification evidence ID has no record: {id}"))?;
            if kind != "independent_observation" && kind != "independent_measurement" {
                return Err(format!(
                    "verification evidence {id} is not an independent observation or measurement"
                ));
            }
            if independence != "independent" {
                return Err(format!(
                    "verification evidence {id} is not explicitly marked independent"
                ));
            }
            if retrieval_state == "unavailable" {
                return Err(format!(
                    "verification evidence {id} is unavailable and cannot substantiate a completed outcome"
                ));
            }
        }
    } else if !verification_ids.is_empty() || !verified_at.is_null() {
        return Err(
            "non-verified outcome must not carry verification IDs or verified_at".to_string(),
        );
    }

    let route = &instance["route"];
    let route_status = route["status"]
        .as_str()
        .ok_or_else(|| "route has no status".to_string())?;
    let route_uris: HashSet<&str> = route["source_references"]
        .as_array()
        .ok_or_else(|| "route source_references must be an array".to_string())?
        .iter()
        .map(|value| {
            value
                .as_str()
                .ok_or_else(|| "route source reference must be a string".to_string())
        })
        .collect::<Result<HashSet<_>, _>>()?;
    let route_evidence_ids = route["source_evidence_ids"]
        .as_array()
        .ok_or_else(|| "route source_evidence_ids must be an array".to_string())?;
    if route_status == "official_contact_source_found" {
        if route_evidence_ids.is_empty() {
            return Err("official contact source status requires evidence IDs".to_string());
        }
        if route["checked_on"].as_str().is_none() {
            return Err("official contact source status requires checked_on date".to_string());
        }
    }
    let mut route_id_set = HashSet::new();
    for value in route_evidence_ids {
        let id = value
            .as_str()
            .ok_or_else(|| "route source evidence ID must be a string".to_string())?;
        if !route_id_set.insert(id) {
            return Err(format!("route source evidence ID is duplicated: {id}"));
        }
        let (kind, _independence, source_uri, retrieval_state) = records
            .get(id)
            .ok_or_else(|| format!("route source evidence ID has no evidence record: {id}"))?;
        if kind != "official_notice" {
            return Err(format!(
                "route source evidence {id} must be an official_notice record"
            ));
        }
        if retrieval_state != "captured" {
            return Err(format!(
                "route source evidence {id} must be a captured official source"
            ));
        }
        if !route_uris.contains(source_uri.as_str()) {
            return Err(format!(
                "route source evidence {id} URI is not listed in route.source_references"
            ));
        }
    }

    if let Some(conflicts) = instance["source_conflicts"].as_array() {
        for conflict in conflicts {
            let conflict_id = conflict["conflict_id"]
                .as_str()
                .unwrap_or("<missing-conflict-id>");
            let ids = conflict["evidence_ids"]
                .as_array()
                .ok_or_else(|| format!("{conflict_id} evidence_ids must be an array"))?;
            if ids.len() < 2 {
                return Err(format!("{conflict_id} requires at least two claim evidence IDs"));
            }
            let mut unique_ids = HashSet::new();
            for value in ids {
                let id = value
                    .as_str()
                    .ok_or_else(|| format!("{conflict_id} evidence ID must be a string"))?;
                if !unique_ids.insert(id) {
                    return Err(format!("{conflict_id} repeats evidence ID {id}"));
                }
                if !records.contains_key(id) {
                    return Err(format!(
                        "{conflict_id} references evidence with no record: {id}"
                    ));
                }
            }

            let resolution_ids = conflict["resolution_evidence_ids"]
                .as_array()
                .ok_or_else(|| {
                    format!("{conflict_id} resolution_evidence_ids must be an array")
                })?;
            if conflict["status"] == "resolved" && resolution_ids.is_empty() {
                return Err(format!(
                    "{conflict_id} cannot be marked resolved without resolution evidence"
                ));
            }
            for value in resolution_ids {
                let id = value.as_str().ok_or_else(|| {
                    format!("{conflict_id} resolution evidence ID must be a string")
                })?;
                if !records.contains_key(id) {
                    return Err(format!(
                        "{conflict_id} references missing resolution evidence: {id}"
                    ));
                }
            }
        }
    }

    if instance["case_state"] == "source_conflict" {
        if status != "conflicted" {
            return Err("source_conflict case must have conflicted outcome status".to_string());
        }
        if instance["review_required"] != true || instance["review_status"] == "not_required" {
            return Err("source_conflict case requires human review".to_string());
        }
        let unresolved = instance["source_conflicts"]
            .as_array()
            .is_some_and(|conflicts| {
                conflicts
                    .iter()
                    .any(|conflict| conflict["status"] == "unresolved")
            });
        if !unresolved {
            return Err(
                "source_conflict case requires at least one unresolved conflict".to_string(),
            );
        }
    }

    Ok(())
}

const INSTRUMENT_SCHEMA: &str = "schemas/life-civic/standard-instrument.schema.json";
const ASSESSMENT_SCHEMA: &str = "schemas/life-civic/applicability-assessment.schema.json";
const CORPUS_PATH: &str = "tests/reference/life-civic-standards/cases.json";
const SERVICE_JOURNEY_SCHEMA: &str = "schemas/life-civic/service-journey-reference.schema.json";
const SERVICE_JOURNEY_CASES: &str = "tests/reference/life-civic-service/cases-v0.3.0.json";
const SERVICE_JOURNEY_POSITIVE: &str =
    "tests/reference/life-civic-service/independently-verified-local-restoration-v0.3.0.json";

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
    let actual_paths: HashSet<&str> = frozen_inputs
        .iter()
        .map(|entry| entry["path"].as_str().expect("frozen path must be a string"))
        .collect();
    assert_eq!(
        actual_paths.len(),
        frozen_inputs.len(),
        "frozen input paths must be unique"
    );
    let expected_paths: HashSet<&str> = [
        INSTRUMENT_SCHEMA,
        ASSESSMENT_SCHEMA,
        CORPUS_PATH,
    ]
    .into_iter()
    .collect();
    assert_eq!(
        actual_paths, expected_paths,
        "manifest must pin the expected schemas and corpus, not unrelated files"
    );
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
    let declared_schema_paths: HashSet<&str> = schema_paths
        .iter()
        .map(|path| path.as_str().expect("schema path must be a string"))
        .collect();
    let expected_schema_paths: HashSet<&str> =
        [INSTRUMENT_SCHEMA, ASSESSMENT_SCHEMA].into_iter().collect();
    assert_eq!(
        declared_schema_paths, expected_schema_paths,
        "manifest schema list must name the exact pinned contracts"
    );
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
fn service_journey_manifest_pins_exact_schema_and_case_bytes() {
    let manifest = read_json("tests/reference/life-civic-service/manifest.json");
    let cases = read_json(SERVICE_JOURNEY_CASES);
    assert_eq!(manifest["dataset_id"], cases["dataset_id"]);
    assert_eq!(manifest["version"], cases["version"]);
    assert_eq!(manifest["cases_file"].as_str(), Some(SERVICE_JOURNEY_CASES));

    let declared_schemas: HashSet<&str> = manifest["schemas"]
        .as_array()
        .expect("service manifest schemas must be an array")
        .iter()
        .map(|value| value.as_str().expect("schema path must be a string"))
        .collect();
    assert_eq!(
        declared_schemas,
        [SERVICE_JOURNEY_SCHEMA].into_iter().collect(),
        "service manifest must identify the exact service-journey schema"
    );

    let frozen_inputs = manifest["frozen_inputs"]
        .as_array()
        .expect("service manifest frozen_inputs must be an array");
    assert_eq!(frozen_inputs.len(), 3);
    let actual_paths: HashSet<&str> = frozen_inputs
        .iter()
        .map(|entry| entry["path"].as_str().expect("frozen path must be a string"))
        .collect();
    assert_eq!(actual_paths.len(), frozen_inputs.len());
    let expected_paths: HashSet<&str> = [
        SERVICE_JOURNEY_SCHEMA,
        SERVICE_JOURNEY_CASES,
        SERVICE_JOURNEY_POSITIVE,
    ]
    .into_iter()
    .collect();
    assert_eq!(actual_paths, expected_paths);

    for entry in frozen_inputs {
        let path = entry["path"].as_str().expect("frozen path must be a string");
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
            "frozen service input changed without updating the manifest: {path}"
        );
    }
}

#[test]
fn scoped_independent_restoration_positive_control_is_accepted() {
    let check = validator(SERVICE_JOURNEY_SCHEMA);
    let fixture = read_json(SERVICE_JOURNEY_POSITIVE);
    assert_eq!(
        fixture["fixture_id"],
        "independently-verified-local-restoration"
    );
    assert_eq!(fixture["expected_schema_valid"], true);
    let instance = &fixture["instance"];
    assert!(check.is_valid(instance), "valid scoped positive control was rejected");
    assert!(
        validate_service_claim_semantics(instance).is_ok(),
        "valid scoped positive control failed cross-field claim semantics"
    );
    assert_eq!(instance["case_state"], "verified_resolved");
    assert_eq!(
        instance["outcome"]["status"],
        "independently_verified_complete"
    );
    let verification_ids = instance["outcome"]["verification_evidence_ids"]
        .as_array()
        .expect("verified outcome must have verification evidence identifiers");
    assert!(!verification_ids.is_empty());
    let evidence = instance["evidence"]
        .as_array()
        .expect("verified outcome must have evidence records");
    for reference_id in verification_ids {
        let reference_id = reference_id
            .as_str()
            .expect("verification evidence identifier must be a string");
        let referenced = evidence
            .iter()
            .find(|item| item["evidence_id"] == reference_id)
            .unwrap_or_else(|| {
                panic!("verification evidence ID has no evidence record: {reference_id}")
            });
        assert!(
            referenced["kind"] == "independent_observation"
                || referenced["kind"] == "independent_measurement",
            "verification evidence must refer to an independent observation or measurement"
        );
        assert_eq!(
            referenced["independence"],
            "independent",
            "verification evidence must explicitly be independent"
        );
    }
    assert_eq!(instance["review_required"], true);
    assert_eq!(instance["review_status"], "completed");
    assert_eq!(instance["effects"]["effects_requested"], false);
    let limitation = instance["evidence"][1]["limitations"]
        .as_str()
        .expect("independent observation must state its limitation");
    assert!(
        limitation.contains("does not establish restoration across the wider service area"),
        "positive control must not generalize one point observation to the whole area"
    );

    let mut dangling_reference = instance.clone();
    dangling_reference["outcome"]["verification_evidence_ids"] = json!(["MISSING-EVIDENCE"]);
    assert!(
        validate_service_claim_semantics(&dangling_reference).is_err(),
        "dangling verification evidence ID must be rejected"
    );

    let mut wrong_kind = instance.clone();
    wrong_kind["outcome"]["verification_evidence_ids"] = json!(["EV-REPORT-005"]);
    assert!(
        validate_service_claim_semantics(&wrong_kind).is_err(),
        "a community report must not qualify as independent verification"
    );

    let mut incomplete_review = instance.clone();
    incomplete_review["review_status"] = json!("requested");
    assert!(
        validate_service_claim_semantics(&incomplete_review).is_err(),
        "independent verification must not pass before human review completes"
    );

    let mut unavailable_verification = instance.clone();
    let unavailable_evidence = unavailable_verification["evidence"]
        .as_array_mut()
        .expect("evidence must be an array")
        .iter_mut()
        .find(|item| item["evidence_id"] == "EV-INDEP-005")
        .expect("independent evidence must exist");
    unavailable_evidence["retrieved_at"] = Value::Null;
    unavailable_evidence["retrieval_state"] = json!("unavailable");
    unavailable_evidence["checked_at"] = json!("2026-10-14T11:00:00Z");
    assert!(
        check.is_valid(&unavailable_verification),
        "unavailable-evidence control must remain schema-valid so the semantic guard is exercised"
    );
    assert!(
        validate_service_claim_semantics(&unavailable_verification).is_err(),
        "unavailable evidence must not substantiate an independently verified completion"
    );

    let mut route_wrong_evidence = instance.clone();
    route_wrong_evidence["route"]["source_evidence_ids"] = json!(["EV-INDEP-005"]);
    assert!(
        validate_service_claim_semantics(&route_wrong_evidence).is_err(),
        "measurement evidence must not masquerade as official contact-source evidence"
    );

    let mut route_dangling_evidence = instance.clone();
    route_dangling_evidence["route"]["source_evidence_ids"] = json!(["MISSING-CONTACT-SOURCE"]);
    assert!(
        validate_service_claim_semantics(&route_dangling_evidence).is_err(),
        "route source evidence IDs must resolve to evidence records"
    );
}

#[test]
fn service_journey_fixtures_preserve_reported_vs_verified_outcomes() {
    let check = validator(SERVICE_JOURNEY_SCHEMA);
    let cases = read_json(SERVICE_JOURNEY_CASES);
    let fixtures = cases["cases"]
        .as_array()
        .expect("service reference cases must be an array");
    assert_eq!(fixtures.len(), 5);

    let mut ids = HashSet::new();
    for fixture in fixtures {
        let fixture_id = fixture["fixture_id"]
            .as_str()
            .expect("fixture_id must be a string");
        assert!(ids.insert(fixture_id), "duplicate service fixture: {fixture_id}");

        let expected = fixture["expected_schema_valid"]
            .as_bool()
            .expect("expected_schema_valid must be boolean");
        let instance = &fixture["instance"];
        assert_eq!(
            instance["effects"]["mode"],
            "effects_disabled",
            "{fixture_id} must keep external effects disabled"
        );
        assert_eq!(
            instance["effects"]["effects_requested"],
            false,
            "{fixture_id} must not request an external effect"
        );
        assert_eq!(
            instance["effects"]["external_action_status"],
            "not_attempted",
            "{fixture_id} must not claim an external action was attempted"
        );
        assert_eq!(
            check.is_valid(instance),
            expected,
            "schema validity mismatch for {fixture_id}"
        );
        assert_eq!(
            validate_service_claim_semantics(instance).is_ok(),
            expected,
            "claim-semantics validity mismatch for {fixture_id}"
        );
    }

    let report = &find_service_fixture(fixtures, "water-report-received")["instance"];
    assert_eq!(report["outcome"]["status"], "unknown");
    assert_eq!(report["case_state"], "received");

    let provider =
        &find_service_fixture(fixtures, "provider-reported-complete-not-verified")["instance"];
    assert_eq!(provider["case_state"], "work_reported_complete");
    assert_eq!(provider["outcome"]["status"], "provider_attested_complete");
    assert_eq!(provider["outcome"]["verified_at"], Value::Null);

    let conflict =
        &find_service_fixture(fixtures, "conflicting-official-water-guidance")["instance"];
    assert_eq!(conflict["case_state"], "source_conflict");
    assert_eq!(
        conflict["route"]["status"],
        "candidate_found",
        "source-target conflict must not be mislabeled as a route conflict"
    );
    assert_eq!(conflict["outcome"]["status"], "conflicted");
    assert_eq!(conflict["review_required"], true);
    assert_eq!(conflict["source_conflicts"].as_array().unwrap().len(), 1);
    let source_conflict = &conflict["source_conflicts"][0];
    assert_eq!(source_conflict["status"], "unresolved");
    assert_eq!(
        source_conflict["evidence_ids"].as_array().unwrap().len(),
        2,
        "source conflict must cite both conflicting claims"
    );
    let conflicting_evidence_ids: HashSet<&str> = source_conflict["evidence_ids"]
        .as_array()
        .unwrap()
        .iter()
        .map(|value| value.as_str().expect("evidence ID must be a string"))
        .collect();
    let actual_evidence_ids: HashSet<&str> = conflict["evidence"]
        .as_array()
        .unwrap()
        .iter()
        .map(|entry| entry["evidence_id"].as_str().expect("evidence_id must be a string"))
        .collect();
    assert!(
        conflicting_evidence_ids.is_subset(&actual_evidence_ids),
        "source conflict must point to evidence actually present in the case"
    );
    assert_eq!(conflict["route"]["source_references"].as_array().unwrap().len(), 3);

    let unavailable =
        &find_service_fixture(fixtures, "unavailable-current-source-requires-review")["instance"];
    assert_eq!(unavailable["case_state"], "human_review_required");
    assert_eq!(unavailable["outcome"]["status"], "unknown");
    assert_eq!(unavailable["review_required"], true);
    assert_eq!(unavailable["evidence"][0]["retrieval_state"], "unavailable");
    assert_eq!(unavailable["evidence"][0]["retrieved_at"], Value::Null);
    assert!(
        unavailable["evidence"][0]["checked_at"].as_str().is_some(),
        "unavailable source must record the failed refresh attempt time"
    );

    let false_verification = &find_service_fixture(
        fixtures,
        "invalid-verified-outcome-with-provider-evidence-only",
    )["instance"];
    assert_eq!(false_verification["outcome"]["status"], "independently_verified_complete");
    assert!(
        false_verification["evidence"]
            .as_array()
            .unwrap()
            .iter()
            .all(|evidence| evidence["kind"] != "independent_observation"
                && evidence["kind"] != "independent_measurement"),
        "negative control must lack independent evidence"
    );
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

    let nist_draft = by_id.get("MYC-LCA-009").expect("NIST OT draft control missing");
    assert_eq!(nist_draft["expected"]["status"], "indeterminate");
    assert_eq!(
        nist_draft["expected"]["required_reason_codes"][0],
        "draft_not_published"
    );
    assert!(
        nist_draft["expected"]["forbidden_claims"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| claim.as_str().unwrap().contains("is a final publication")),
        "NIST draft control must forbid promoting the initial public draft to final"
    );

    let unodc_guidance = by_id
        .get("MYC-LCA-010")
        .expect("UNODC prevention-guidance scope control missing");
    assert_eq!(unodc_guidance["expected"]["status"], "unknown");
    assert_eq!(
        unodc_guidance["expected"]["required_reason_codes"][0],
        "context_insufficient"
    );
    assert!(
        unodc_guidance["expected"]["forbidden_claims"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| claim.as_str().unwrap().contains("reduce crime")),
        "UNODC guidance control must not imply a robot intervention has a proven effect"
    );

    let nij_evidence = by_id
        .get("MYC-LCA-011")
        .expect("NIJ employment-crime scope control missing");
    assert_eq!(nij_evidence["expected"]["status"], "unknown");
    assert_eq!(
        nij_evidence["expected"]["required_reason_codes"][0],
        "context_insufficient"
    );
    assert!(
        nij_evidence["expected"]["forbidden_claims"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| claim.as_str().unwrap().contains("proves the local program")),
        "NIJ evidence control must prevent unsupported local causal generalization"
    );

    let robot_scope = by_id
        .get("MYC-LCA-012")
        .expect("industrial-versus-service robot scope control missing");
    assert_eq!(robot_scope["expected"]["status"], "unknown");
    assert_eq!(
        robot_scope["expected"]["required_reason_codes"][0],
        "context_insufficient"
    );
    assert!(
        robot_scope["expected"]["forbidden_claims"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| {
                claim
                    .as_str()
                    .unwrap()
                    .contains("fully covers a public-access service robot")
            }),
        "industrial robot-cell standard must not be generalized to a public-facing service robot"
    );
}
