//! Independent, effects-disabled checks for the life/civic reference contracts.
//!
//! This checker validates schema shape and corpus invariants. It does not determine
//! legal applicability, certify conformance, or authorize service actions.

use serde_json::{Value, json};
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
            if retrieval_state != "captured" {
                return Err(format!(
                    "verification evidence {id} must be captured; {retrieval_state} evidence cannot substantiate a completed outcome"
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
                return Err(format!(
                    "{conflict_id} requires at least two claim evidence IDs"
                ));
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
                .ok_or_else(|| format!("{conflict_id} resolution_evidence_ids must be an array"))?;
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
const GPSJS_QUALITY_SCHEMA: &str = "schemas/life-civic/gpsjs-survey-quality.schema.json";
const CORPUS_PATH: &str = "tests/reference/life-civic-standards/cases.json";
const SERVICE_JOURNEY_SCHEMA: &str = "schemas/life-civic/service-journey-reference.schema.json";
const SERVICE_JOURNEY_CASES: &str = "tests/reference/life-civic-service/cases-v0.6.0.json";
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
    assert_eq!(
        frozen_inputs.len(),
        4,
        "all schema/corpus inputs must be pinned"
    );
    let actual_paths: HashSet<&str> = frozen_inputs
        .iter()
        .map(|entry| {
            entry["path"]
                .as_str()
                .expect("frozen path must be a string")
        })
        .collect();
    assert_eq!(
        actual_paths.len(),
        frozen_inputs.len(),
        "frozen input paths must be unique"
    );
    let expected_paths: HashSet<&str> = [
        INSTRUMENT_SCHEMA,
        ASSESSMENT_SCHEMA,
        GPSJS_QUALITY_SCHEMA,
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
    assert_eq!(schema_paths.len(), 3);
    let declared_schema_paths: HashSet<&str> = schema_paths
        .iter()
        .map(|path| path.as_str().expect("schema path must be a string"))
        .collect();
    let expected_schema_paths: HashSet<&str> =
        [INSTRUMENT_SCHEMA, ASSESSMENT_SCHEMA, GPSJS_QUALITY_SCHEMA]
            .into_iter()
            .collect();
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
        .map(|entry| {
            entry["path"]
                .as_str()
                .expect("frozen path must be a string")
        })
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
        let path = entry["path"]
            .as_str()
            .expect("frozen path must be a string");
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
fn scoped_independent_restoration_requires_captured_evidence() {
    let check = validator(SERVICE_JOURNEY_SCHEMA);
    let fixture = read_json(SERVICE_JOURNEY_POSITIVE);
    assert_eq!(
        fixture["fixture_id"],
        "independently-verified-local-restoration"
    );
    assert_eq!(fixture["expected_schema_valid"], true);
    let instance = &fixture["instance"];
    assert!(
        check.is_valid(instance),
        "schema-valid synthetic control was rejected"
    );
    assert!(
        validate_service_claim_semantics(instance).is_err(),
        "synthetic evidence must not substantiate a real independently verified completion"
    );

    // This in-memory-only control exercises the captured-evidence path without
    // changing the repository fixture's honest synthetic provenance label.
    let mut captured_verification = instance.clone();
    let captured_evidence = captured_verification["evidence"]
        .as_array_mut()
        .expect("evidence must be an array")
        .iter_mut()
        .find(|item| item["evidence_id"] == "EV-INDEP-005")
        .expect("independent evidence must exist");
    captured_evidence["retrieved_at"] = json!("2026-10-14T19:54:08+02:00");
    captured_evidence["retrieval_state"] = json!("captured");
    captured_evidence["checked_at"] = json!("2026-10-14T19:54:08+02:00");
    captured_evidence["limitations"] = json!(
        "In-memory semantic test control only; no external independent measurement was retrieved."
    );
    assert!(check.is_valid(&captured_verification));
    assert!(
        validate_service_claim_semantics(&captured_verification).is_ok(),
        "captured independent evidence with completed review should satisfy the semantic guard"
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
            referenced["independence"], "independent",
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

    let mut dangling_reference = captured_verification.clone();
    dangling_reference["outcome"]["verification_evidence_ids"] = json!(["MISSING-EVIDENCE"]);
    assert!(
        validate_service_claim_semantics(&dangling_reference).is_err(),
        "dangling verification evidence ID must be rejected"
    );

    let mut wrong_kind = captured_verification.clone();
    wrong_kind["outcome"]["verification_evidence_ids"] = json!(["EV-REPORT-005"]);
    assert!(
        validate_service_claim_semantics(&wrong_kind).is_err(),
        "a community report must not qualify as independent verification"
    );

    let mut incomplete_review = captured_verification.clone();
    incomplete_review["review_status"] = json!("requested");
    assert!(
        validate_service_claim_semantics(&incomplete_review).is_err(),
        "independent verification must not pass before human review completes"
    );

    let mut unavailable_verification = captured_verification.clone();
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

    let mut route_wrong_evidence = captured_verification.clone();
    route_wrong_evidence["route"]["source_evidence_ids"] = json!(["EV-INDEP-005"]);
    assert!(
        validate_service_claim_semantics(&route_wrong_evidence).is_err(),
        "measurement evidence must not masquerade as official contact-source evidence"
    );

    let mut route_dangling_evidence = captured_verification.clone();
    route_dangling_evidence["route"]["source_evidence_ids"] = json!(["MISSING-CONTACT-SOURCE"]);
    assert!(
        validate_service_claim_semantics(&route_dangling_evidence).is_err(),
        "route source evidence IDs must resolve to evidence records"
    );
}

#[test]
fn synthetic_johannesburg_project_claims_do_not_prove_live_service_or_verified_outcomes() {
    let check = validator(SERVICE_JOURNEY_SCHEMA);
    let cases = read_json(SERVICE_JOURNEY_CASES);
    let fixtures = cases["cases"]
        .as_array()
        .expect("service reference cases must be an array");

    for (fixture_id, case_id, source_uri, source_date, prohibited_probe) in [
        (
            "city-water-depot-closeout-not-operational-acceptance",
            "LCSV-JHB-008",
            "https://joburg.org.za/media_/Newsroom/Pages/2026-News-Articles/Joburg-Water-depot-upgrade-to-strengthen-service-delivery.aspx",
            "2026-10-12",
            "fully commissioned",
        ),
        (
            "city-braamfontein-traffic-upgrade-not-live-uptime",
            "LCSV-JHB-009",
            "https://joburg.org.za/media_/Newsroom/Pages/2026-News-Articles/Braamfontein-traffic-upgrades-set-to-make-journeys-safer-and-smoother.aspx",
            "2026-10-02",
            "operating correctly today",
        ),
        (
            "city-malvern-clinic-reopening-not-current-availability",
            "LCSV-JHB-010",
            "https://joburg.org.za/media_/Newsroom/Pages/2026-News-Articles/Refurbished-Malvern-Clinic-restores-access-to-healthcare.aspx",
            "2026-10-14",
            "fully staffed or adequately supplied today",
        ),
    ] {
        let fixture = find_service_fixture(fixtures, fixture_id);
        assert_eq!(fixture["expected_schema_valid"], true);
        let instance = &fixture["instance"];
        assert_eq!(instance["case_id"], case_id);
        assert_eq!(instance["scope"]["service_class"], "general_municipal");
        assert_eq!(instance["case_state"], "human_review_required");
        assert_eq!(instance["outcome"]["status"], "unknown");
        assert!(
            instance["outcome"]["verification_evidence_ids"]
                .as_array()
                .unwrap()
                .is_empty()
        );
        assert_eq!(instance["outcome"]["verified_at"], Value::Null);
        assert_eq!(instance["effects"]["mode"], "effects_disabled");
        assert_eq!(instance["effects"]["effects_requested"], false);
        assert!(
            check.is_valid(instance),
            "{fixture_id} must satisfy service journey schema"
        );
        assert!(
            validate_service_claim_semantics(instance).is_ok(),
            "{fixture_id} must satisfy service claim semantics"
        );

        let evidence = &instance["evidence"][0];
        assert_eq!(evidence["kind"], "official_notice");
        assert_eq!(evidence["source_uri"], source_uri);
        assert_eq!(evidence["observed_on"], source_date);
        assert_eq!(evidence["independence"], "unknown");
        assert_eq!(evidence["retrieval_state"], "synthetic_fixture");
        assert_eq!(evidence["retrieved_at"], Value::Null);
        assert_eq!(evidence["checked_at"], Value::Null);
        assert!(
            evidence["limitations"]
                .as_str()
                .unwrap()
                .contains("not captured as evidence"),
            "{fixture_id} must retain its non-capture limitation"
        );
        let prohibited = instance["claim_ceiling"]["prohibited_inferences"]
            .as_array()
            .expect("prohibited inferences must be an array");
        assert!(
            prohibited.iter().any(|claim| claim
                .as_str()
                .is_some_and(|text| text.contains(prohibited_probe))),
            "{fixture_id} is missing its key prohibited inference"
        );
    }
}

#[test]
fn water_response_target_is_not_turnaround_or_restoration_target() {
    let check = validator(SERVICE_JOURNEY_SCHEMA);
    let cases = read_json(SERVICE_JOURNEY_CASES);
    let fixtures = cases["cases"]
        .as_array()
        .expect("service reference cases must be an array");
    let fixture = find_service_fixture(
        fixtures,
        "water-response-target-not-restoration-or-turnaround",
    );
    assert_eq!(fixture["expected_schema_valid"], true);
    let instance = &fixture["instance"];
    assert_eq!(instance["case_id"], "LCSV-WATER-011");
    assert_eq!(instance["case_state"], "human_review_required");
    assert_eq!(instance["outcome"]["status"], "unknown");
    assert_eq!(instance["review_required"], true);
    assert_eq!(instance["review_status"], "requested");
    assert_eq!(instance["route"]["status"], "unknown");
    assert!(instance["source_conflicts"].as_array().unwrap().is_empty());
    assert!(check.is_valid(instance));
    assert!(validate_service_claim_semantics(instance).is_ok());

    let evidence = instance["evidence"]
        .as_array()
        .expect("clock claims must remain separate evidence entries");
    assert_eq!(evidence.len(), 2);
    assert_eq!(
        evidence[0]["source_uri"],
        "https://www.johannesburgwater.co.za/services/"
    );
    assert!(
        evidence[0]["claim_scope"]
            .as_str()
            .unwrap()
            .contains("respond within 12 hours")
    );
    assert_eq!(
        evidence[1]["source_uri"],
        "https://www.johannesburgwater.co.za/daily-water-notices/"
    );
    let turnaround = evidence[1]["claim_scope"].as_str().unwrap();
    for category_value in [
        "sewer 48 hours",
        "water outage 24 hours",
        "leaking meter 72 hours",
        "burst meter 24 hours",
        "water leak 48 hours",
        "major burst pipe 48 hours",
    ] {
        assert!(
            turnaround.contains(category_value),
            "Daily Water Notices evidence is missing {category_value}"
        );
    }
    for record in evidence {
        assert_eq!(record["retrieval_state"], "synthetic_fixture");
        assert_eq!(record["retrieved_at"], Value::Null);
        assert_eq!(record["checked_at"], Value::Null);
        let limitation = record["limitations"].as_str().unwrap();
        assert!(
            limitation.contains("raw-source capture")
                || limitation.contains("retrieved page bytes"),
            "synthetic evidence must not be represented as raw-source capture"
        );
    }

    let prohibited = instance["claim_ceiling"]["prohibited_inferences"]
        .as_array()
        .expect("prohibited inferences must be recorded");
    assert!(prohibited.iter().any(|claim| claim
        .as_str()
        .is_some_and(|text| text.contains("repair-completion or restoration target"))));
    assert!(prohibited.iter().any(|claim| claim
        .as_str()
        .is_some_and(|text| text.contains("metric labels differ"))));
}

#[test]
fn service_journey_fixtures_preserve_reported_vs_verified_outcomes() {
    let check = validator(SERVICE_JOURNEY_SCHEMA);
    let cases = read_json(SERVICE_JOURNEY_CASES);
    let fixtures = cases["cases"]
        .as_array()
        .expect("service reference cases must be an array");
    assert_eq!(fixtures.len(), 10);

    let mut ids = HashSet::new();
    for fixture in fixtures {
        let fixture_id = fixture["fixture_id"]
            .as_str()
            .expect("fixture_id must be a string");
        assert!(
            ids.insert(fixture_id),
            "duplicate service fixture: {fixture_id}"
        );

        let expected = fixture["expected_schema_valid"]
            .as_bool()
            .expect("expected_schema_valid must be boolean");
        let instance = &fixture["instance"];
        assert_eq!(
            instance["effects"]["mode"], "effects_disabled",
            "{fixture_id} must keep external effects disabled"
        );
        assert_eq!(
            instance["effects"]["effects_requested"], false,
            "{fixture_id} must not request an external effect"
        );
        assert_eq!(
            instance["effects"]["external_action_status"], "not_attempted",
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
        conflict["route"]["status"], "candidate_found",
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
        .map(|entry| {
            entry["evidence_id"]
                .as_str()
                .expect("evidence_id must be a string")
        })
        .collect();
    assert!(
        conflicting_evidence_ids.is_subset(&actual_evidence_ids),
        "source conflict must point to evidence actually present in the case"
    );
    assert_eq!(
        conflict["route"]["source_references"]
            .as_array()
            .unwrap()
            .len(),
        3
    );

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

    let cross_utility =
        &find_service_fixture(fixtures, "cross-utility-statement-not-incident-cause")["instance"];
    assert_eq!(cross_utility["case_state"], "human_review_required");
    assert_eq!(cross_utility["route"]["status"], "unknown");
    assert_eq!(
        cross_utility["route"]["source_evidence_ids"]
            .as_array()
            .unwrap()
            .len(),
        0
    );
    assert_eq!(cross_utility["outcome"]["status"], "unknown");
    assert_eq!(cross_utility["review_required"], true);
    assert_eq!(cross_utility["review_status"], "requested");
    assert_eq!(cross_utility["effects"]["effects_requested"], false);
    let statement = &cross_utility["evidence"][0];
    assert_eq!(statement["kind"], "official_notice");
    assert_eq!(statement["retrieval_state"], "synthetic_fixture");
    assert_eq!(statement["retrieved_at"], Value::Null);
    assert_eq!(statement["checked_at"], Value::Null);
    assert!(
        statement["limitations"].as_str().unwrap().contains(
            "not evidence that a specific resident's interruption was caused by electricity"
        ),
        "public cross-utility context must not become incident-level causal evidence"
    );
    assert!(
        cross_utility["claim_ceiling"]["prohibited_inferences"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| claim.as_str().unwrap().contains("caused by a power trip")),
        "negative control must forbid incident-level power-causality inference"
    );
    assert!(
        cross_utility["claim_ceiling"]["prohibited_inferences"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| claim.as_str().unwrap().contains("restored now")),
        "negative control must forbid asserting current restoration from a historical statement"
    );

    let false_verification = &find_service_fixture(
        fixtures,
        "invalid-verified-outcome-with-provider-evidence-only",
    )["instance"];
    assert_eq!(
        false_verification["outcome"]["status"],
        "independently_verified_complete"
    );
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
fn all_declared_schemas_are_valid_json_schema_documents() {
    let _instrument = validator(INSTRUMENT_SCHEMA);
    let _assessment = validator(ASSESSMENT_SCHEMA);
    let _gpsjs_quality = validator(GPSJS_QUALITY_SCHEMA);
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
    let valid_reasons: HashSet<&str> = schema["properties"]["result"]["properties"]["reason_codes"]
        ["items"]["enum"]
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
        assert!(
            valid_statuses.contains(status),
            "{id} uses unknown status {status}"
        );

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
        assert!(
            !forbidden.is_empty(),
            "{id} has no prohibited-claim controls"
        );
        assert!(
            case["synthetic"].is_boolean(),
            "{id} must say whether it is synthetic"
        );
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

    let draft = by_id
        .get("MYC-LCA-002")
        .expect("ISO 37120 draft control missing");
    assert_eq!(draft["expected"]["status"], "indeterminate");
    assert_eq!(
        draft["expected"]["required_reason_codes"][0],
        "draft_not_published"
    );

    let lifecycle = by_id
        .get("MYC-LCA-003")
        .expect("ISO 37101 lifecycle control missing");
    assert!(
        lifecycle["expected"]["forbidden_claims"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| {
                claim
                    .as_str()
                    .unwrap()
                    .contains("withdrawn solely because a replacement is being drafted")
            })
    );

    let registry = by_id
        .get("MYC-LCA-007")
        .expect("registry-authority control missing");
    assert!(
        registry["expected"]["forbidden_claims"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| claim.as_str().unwrap().contains("authorizes the action"))
    );

    let transparency = by_id
        .get("MYC-LCA-008")
        .expect("transparency/truth control missing");
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

    let nist_draft = by_id
        .get("MYC-LCA-009")
        .expect("NIST OT draft control missing");
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

    let statssa = by_id
        .get("MYC-LCA-013")
        .expect("Stats SA national-statistic scope control missing");
    assert_eq!(statssa["expected"]["status"], "unknown");
    assert_eq!(
        statssa["expected"]["required_reason_codes"][0],
        "context_insufficient"
    );
    assert!(
        statssa["expected"]["forbidden_claims"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| claim.as_str().unwrap().contains("causal crime coefficient")),
        "national QLFS descriptive statistics must not be treated as a crime coefficient"
    );

    let decent_work = by_id
        .get("MYC-LCA-014")
        .expect("ILO decent-work threshold-scope control missing");
    assert_eq!(decent_work["expected"]["status"], "unknown");
    assert_eq!(
        decent_work["expected"]["required_reason_codes"][0],
        "context_insufficient"
    );
    assert!(
        decent_work["expected"]["forbidden_claims"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| claim
                .as_str()
                .unwrap()
                .contains("universal 0.80 fair-pay adequacy threshold")),
        "ILO measurement framework must not be treated as a source for the synthetic numeric threshold"
    );

    let victimization_reporting = by_id
        .get("MYC-LCA-015")
        .expect("victimization-versus-police-report observability control missing");
    assert_eq!(victimization_reporting["expected"]["status"], "unknown");
    assert_eq!(
        victimization_reporting["expected"]["required_reason_codes"][0],
        "context_insufficient"
    );
    assert!(
        victimization_reporting["expected"]["forbidden_claims"]
            .as_array()
            .unwrap()
            .iter()
            .any(|claim| claim
                .as_str()
                .unwrap()
                .contains("lower police-report counts alone")),
        "police-record changes alone must not be treated as proof of reduced victimization"
    );

    let latest_victimization = by_id
        .get("MYC-LCA-016")
        .expect("latest GPSJS time-series boundary missing");
    assert_eq!(latest_victimization["expected"]["status"], "unknown");
    assert_eq!(
        latest_victimization["input_projection"]["version"],
        "GPSJS 2025/26 Victims of Crime Release; published 2026-09-01"
    );
    assert_eq!(
        latest_victimization["expected"]["required_reason_codes"][0],
        "context_insufficient"
    );
    let latest_forbidden = latest_victimization["expected"]["forbidden_claims"]
        .as_array()
        .expect("latest GPSJS prohibited claims must be an array");
    assert!(
        latest_forbidden
            .iter()
            .any(|claim| { claim.as_str().unwrap().contains("43.4% to 48.2%") }),
        "latest GPSJS control must preserve the reporting-rate trend without a false causal inference"
    );
    assert!(
        latest_forbidden.iter().any(|claim| {
            claim
                .as_str()
                .unwrap()
                .contains("1.493 million to 1.549 million")
        }),
        "latest GPSJS control must keep incident count distinct from affected-household count"
    );
    assert_eq!(
        corpus["version"], "0.5.0",
        "time-series boundary must be part of the versioned current corpus"
    );
}

#[test]
fn gpsjs_precision_negative_controls_remain_present() {
    let corpus = read_json(CORPUS_PATH);
    let cases = corpus["cases"].as_array().expect("cases array");
    let precision = cases
        .iter()
        .find(|case| case["case_id"] == "MYC-LCA-017")
        .expect("GPSJS survey-precision control missing");

    assert_eq!(precision["expected"]["status"], "unknown");
    assert_eq!(
        precision["expected"]["required_reason_codes"][0],
        "context_insufficient"
    );
    assert_eq!(
        precision["input_projection"]["version"],
        "GPSJS 2025/26 Victims of Crime Release technical notes; published 2026-09-01"
    );
    let rationale = precision["rationale"]
        .as_str()
        .expect("precision rationale must be a string");
    assert!(rationale.contains("85.57%"));
    assert!(rationale.contains("76.97%"));
    assert!(rationale.contains("80% minimum-acceptable"));

    let quality_validator = validator(GPSJS_QUALITY_SCHEMA);
    assert!(
        quality_validator.is_valid(&precision["survey_quality"]),
        "GPSJS survey quality metadata must satisfy its dedicated schema"
    );
    let mut invalid_quality = precision["survey_quality"].clone();
    invalid_quality["response_rate_percent"]["national"] = json!(100.01);
    assert!(
        !quality_validator.is_valid(&invalid_quality),
        "response rate above 100% must fail the schema"
    );
    let mut invalid_precision_scope = precision["survey_quality"].clone();
    let invalid_precision_scope_limits = &mut invalid_precision_scope["precision_scope_limits"];
    invalid_precision_scope_limits["table_41_prevalence_cv_is_precision_for_reporting_rate_change"] =
        json!(true);
    assert!(
        !quality_validator.is_valid(&invalid_precision_scope),
        "Table 41 prevalence CV must not be accepted as precision for police-reporting proportions"
    );

    let quality = precision["survey_quality"]
        .as_object()
        .expect("survey quality diagnostics must be structured");
    assert_eq!(quality["response_rate_percent"]["national"].as_f64(), Some(85.57));
    assert_eq!(quality["response_rate_percent"]["gauteng"].as_f64(), Some(76.97));
    assert_eq!(
        quality["response_rate_percent"]["city_of_johannesburg"].as_f64(),
        Some(73.72)
    );
    assert_eq!(
        quality["response_rate_percent"]["city_of_tshwane"].as_f64(),
        Some(67.54)
    );
    assert_eq!(
        quality["geographic_areas_below_80_percent_minimum_acceptable"],
        4
    );
    assert_eq!(
        quality["threshold_interpretation"],
        "response_rate_quality_diagnostic_not_significance_test"
    );
    assert_eq!(
        quality["requires_estimate_specific_precision_for_significance"],
        true
    );
    assert_eq!(
        quality["requires_denominator_and_indicator_compatibility"],
        true
    );
    assert_eq!(
        quality["requires_geographic_and_time_scope_match"],
        true
    );
    assert_eq!(
        quality["requires_local_validation_for_intervention_transfer"],
        true
    );
    let accepted_precision = quality["accepted_precision_evidence"]
        .as_array()
        .expect("accepted precision evidence must be an array");
    for required in [
        "standard_error",
        "confidence_interval",
        "coefficient_of_variation",
        "design_effect_or_official_precision_note",
    ] {
        assert!(
            accepted_precision.iter().any(|item| item == required),
            "survey precision contract missing {required}"
        );
    }

    let precision_examples = quality["published_precision_examples"]
        .as_array()
        .expect("published precision examples must be an array");
    let prevalence_2024_25 = precision_examples
        .iter()
        .find(|item| item["release"] == "GPSJS 2024/25")
        .expect("2024/25 Table 41 example missing");
    let prevalence_2025_26 = precision_examples
        .iter()
        .find(|item| item["release"] == "GPSJS 2025/26")
        .expect("2025/26 Table 41 example missing");
    for (example, estimate) in [(prevalence_2024_25, 43.1), (prevalence_2025_26, 46.0)] {
        assert_eq!(example["table"], "Table 41");
        assert_eq!(
            example["indicator"],
            "household_housebreaking_experience_past_12_months"
        );
        assert_eq!(example["estimate_percent"].as_f64(), Some(estimate));
        assert_eq!(example["coefficient_of_variation_percent"].as_f64(), Some(3.0));
        assert!(
            example["source_uri"]
                .as_str()
                .unwrap()
                .starts_with("https://")
        );
    }
    assert_eq!(
        quality["precision_scope_limits"]["reporting_rate_change_percent_points"],
        json!(4.8)
    );
    assert_eq!(
        quality["precision_scope_limits"]["year_over_year_significance_established"],
        false
    );
    assert_eq!(
        quality["precision_scope_limits"]
            ["table_41_prevalence_cv_is_precision_for_reporting_rate_change"],
        false
    );


    let forbidden = precision["expected"]["forbidden_claims"]
        .as_array()
        .expect("precision prohibited claims must be an array");
    assert!(
        forbidden.iter().any(|claim| {
            claim
                .as_str()
                .unwrap()
                .contains("statistically significant because rounded point estimates differ")
        }),
        "precision control must forbid significance claims from rounded point estimates alone"
    );
    assert!(
        forbidden.iter().any(|claim| {
            claim
                .as_str()
                .unwrap()
                .contains("response rate was 76.97%")
        }),
        "precision control must not declare every lower-response-area estimate invalid"
    );
    assert!(
        forbidden.iter().any(|claim| {
            claim
                .as_str()
                .unwrap()
                .contains("confidence intervals")
        }),
        "precision control must preserve estimate-specific uncertainty requirements"
    );
    assert_eq!(
        corpus["version"], "0.5.0",
        "survey-precision boundary must be part of the versioned current corpus"
    );
}
