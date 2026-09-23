use mycelix_data_privacy::*;
use serde::de::DeserializeOwned;
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

const FIXTURE: &str =
    include_str!("../../../docs/embodied/fixtures/EMB_DATA_PRIVACY_PURPOSE_V0_1.json");

fn parse<T: DeserializeOwned>(value: &Value) -> T {
    serde_json::from_value(value.clone()).expect("fixture vocabulary should deserialize")
}

fn string(value: &Value) -> String {
    value.as_str().expect("expected string").to_owned()
}

fn set_from_array<T: DeserializeOwned + Ord>(value: &Value) -> BTreeSet<T> {
    value
        .as_array()
        .expect("expected array")
        .iter()
        .map(parse)
        .collect()
}

fn string_set(value: &Value) -> BTreeSet<String> {
    value
        .as_array()
        .expect("expected array")
        .iter()
        .map(string)
        .collect()
}

fn purpose_map(value: &Value) -> BTreeMap<PrivacyPurpose, PurposeDisposition> {
    value
        .as_object()
        .expect("expected purpose map")
        .iter()
        .map(|(purpose, disposition)| {
            (
                serde_json::from_value(Value::String(purpose.clone()))
                    .expect("known purpose"),
                parse(disposition),
            )
        })
        .collect()
}

fn obligations(value: Option<&Value>) -> BTreeSet<PrivacyObligation> {
    value.map(set_from_array).unwrap_or_default()
}

fn nonclaims(value: Option<&Value>) -> BTreeSet<PrivacyNonclaim> {
    value.map(set_from_array).unwrap_or_default()
}

fn transformations(case: &Value) -> Vec<ReviewedTransformationV1> {
    case["transformations"]
        .as_array()
        .expect("transformations array")
        .iter()
        .map(|row| ReviewedTransformationV1 {
            transform_ref: string(&row["id"]),
            review_state: parse(&row["review_state"]),
            output_modalities: set_from_array(&row["output_modalities"]),
        })
        .collect()
}

fn transformation_refs(case: &Value) -> BTreeSet<String> {
    case["transformations"]
        .as_array()
        .expect("transformations array")
        .iter()
        .map(|row| string(&row["id"]))
        .collect()
}

fn evidence_for_case(case: &Value) -> Vec<ReviewedPrivacyPurposeEvidenceV1> {
    let root = &case["reviewed_privacy_purpose_evidence"];
    let participant_evidence = parse(&root["participant_evidence"]);
    let bystander_evidence = parse(&root["bystander_evidence"]);
    let root_currentness = parse(&root["currentness"]);
    let transform_refs = transformation_refs(case);

    if let Some(records) = root.get("purpose_evidence_records").and_then(Value::as_array) {
        return records
            .iter()
            .map(|row| ReviewedPrivacyPurposeEvidenceV1 {
                subject: string(&row["subject"]),
                evidence_ref: string(&row["evidence_ref"]),
                currentness: parse(&row["currentness"]),
                participant_evidence,
                bystander_evidence,
                purpose_dispositions: [(parse(&row["purpose"]), parse(&row["disposition"]))]
                    .into_iter()
                    .collect(),
                obligations: obligations(root.get("obligations")),
                nonclaims: nonclaims(root.get("nonclaims")),
                transformation_refs: transform_refs.clone(),
            })
            .collect();
    }

    if let Some(rows) = root.get("per_source").and_then(Value::as_array) {
        return rows
            .iter()
            .map(|row| ReviewedPrivacyPurposeEvidenceV1 {
                subject: string(&row["subject"]),
                evidence_ref: format!(
                    "fixture:{}:{}",
                    case["id"].as_str().expect("case id"),
                    row["subject"].as_str().expect("subject")
                ),
                currentness: root_currentness,
                participant_evidence,
                bystander_evidence,
                purpose_dispositions: purpose_map(&row["purpose_dimensions"]),
                obligations: obligations(row.get("obligations")),
                nonclaims: nonclaims(row.get("nonclaims")),
                transformation_refs: transform_refs.clone(),
            })
            .collect();
    }

    vec![ReviewedPrivacyPurposeEvidenceV1 {
        subject: string(&root["subject"]),
        evidence_ref: format!(
            "fixture:{}:root",
            case["id"].as_str().expect("case id")
        ),
        currentness: root_currentness,
        participant_evidence,
        bystander_evidence,
        purpose_dispositions: purpose_map(&root["purpose_dimensions"]),
        obligations: obligations(root.get("obligations")),
        nonclaims: nonclaims(root.get("nonclaims")),
        transformation_refs: transform_refs,
    }]
}

fn request_for_case(case: &Value) -> IntendedPrivacyPurposeUseV1 {
    IntendedPrivacyPurposeUseV1 {
        subjects: string_set(&case["subjects"]),
        dataset_split_role: parse(&case["split_role"]),
        source_modalities: set_from_array(&case["source_modalities"]),
        requested_purpose: parse(&case["requested_purpose"]),
        requested_modalities: set_from_array(&case["requested_modalities"]),
        excluded_modalities: set_from_array(&case["excluded_modalities"]),
        reviewed_transformations: transformations(case),
        external_prerequisites: set_from_array(&case["external_prerequisites"]),
    }
}

fn retained_obligation_set(
    assessment: &PrivacyPurposeCompatibilityAssessmentV1,
) -> BTreeSet<PrivacyObligation> {
    assessment
        .retained_obligations()
        .iter()
        .map(|row| row.obligation())
        .collect()
}

#[test]
fn exact_qualified_c01_c20_corpus_replays() {
    let document: Value = serde_json::from_str(FIXTURE).expect("valid frozen fixture");
    assert_eq!(
        document["schema"],
        "mycelix:emb-data-privacy-purpose-corpus:v0.1"
    );
    assert_eq!(document["authority"], "SyntheticPrivacyPurposeSemanticsOnly");
    for field in [
        "legal_compliance_authority",
        "consent_validity_authority",
        "training_execution_authority",
        "publication_authority",
        "physical_execution_authority",
        "reidentification_impossible_authority",
    ] {
        assert_eq!(document[field], false, "authority flag {field} must remain false");
    }

    let cases = document["cases"].as_array().expect("cases array");
    assert_eq!(cases.len(), 20);

    for case in cases {
        let evidence = evidence_for_case(case);
        let request = request_for_case(case);
        let assessment = assess_privacy_purpose_compatibility(&evidence, &request)
            .unwrap_or_else(|error| panic!("{}: {error}", case["id"]));

        assert_eq!(
            assessment.state().as_str(),
            case["expected_state"].as_str().expect("expected state"),
            "state mismatch for {}",
            case["id"]
        );
        assert_eq!(
            retained_obligation_set(&assessment),
            set_from_array(&case["expected_obligations"]),
            "obligation mismatch for {}",
            case["id"]
        );
        assert_eq!(
            assessment.excluded_modalities(),
            &set_from_array(&case["excluded_modalities"]),
            "excluded modality mismatch for {}",
            case["id"]
        );
        assert_eq!(
            assessment.external_prerequisites(),
            &set_from_array(&case["external_prerequisites"]),
            "external prerequisite mismatch for {}",
            case["id"]
        );
    }
}

#[test]
fn contradiction_and_multisource_results_are_evidence_order_invariant() {
    let document: Value = serde_json::from_str(FIXTURE).expect("valid frozen fixture");
    let cases = document["cases"].as_array().expect("cases array");

    for id in [
        "C09-contradictory-purpose-evidence",
        "C18-multisource-deletion-obligation-union",
    ] {
        let case = cases
            .iter()
            .find(|case| case["id"] == id)
            .expect("named fixture case");
        let request = request_for_case(case);
        let mut evidence = evidence_for_case(case);
        let forward = assess_privacy_purpose_compatibility(&evidence, &request)
            .expect("forward evidence should assess");
        evidence.reverse();
        let reverse = assess_privacy_purpose_compatibility(&evidence, &request)
            .expect("reversed evidence should assess");
        assert_eq!(forward, reverse, "evidence order changed {id}");
    }
}

#[test]
fn terms_reference_has_no_decision_path_and_data_minimization_is_preserved() {
    let document: Value = serde_json::from_str(FIXTURE).expect("valid frozen fixture");
    let cases = document["cases"].as_array().expect("cases array");

    let c19 = cases
        .iter()
        .find(|case| case["id"] == "C19-terms-compatibility-cannot-manufacture-privacy")
        .expect("C19");
    assert!(c19["reviewed_privacy_purpose_evidence"]
        .get("terms_compatibility_ref")
        .is_some());
    let result = assess_privacy_purpose_compatibility(
        &evidence_for_case(c19),
        &request_for_case(c19),
    )
    .expect("C19 should assess");
    assert_eq!(
        result.state(),
        PrivacyPurposeAssessmentState::HumanReviewRequired
    );

    let c03 = cases
        .iter()
        .find(|case| case["id"] == "C03-unneeded-audio-explicitly-excluded")
        .expect("C03");
    let c03_result = assess_privacy_purpose_compatibility(
        &evidence_for_case(c03),
        &request_for_case(c03),
    )
    .expect("C03 should assess");
    assert!(c03_result.source_modalities().contains(&Modality::Audio));
    assert!(!c03_result.requested_modalities().contains(&Modality::Audio));
    assert!(c03_result.excluded_modalities().contains(&Modality::Audio));

    let c11 = cases
        .iter()
        .find(|case| case["id"] == "C11-deidentification-transform-retains-nonclaim")
        .expect("C11");
    let c11_result = assess_privacy_purpose_compatibility(
        &evidence_for_case(c11),
        &request_for_case(c11),
    )
    .expect("C11 should assess");
    assert!(c11_result.retained_nonclaims().iter().any(|row| {
        row.nonclaim() == PrivacyNonclaim::ReidentificationImpossible
    }));

    let c20 = cases
        .iter()
        .find(|case| case["id"] == "C20-compatible-never-creates-publication-or-actuation-authority")
        .expect("C20");
    let c20_result = assess_privacy_purpose_compatibility(
        &evidence_for_case(c20),
        &request_for_case(c20),
    )
    .expect("C20 should assess");
    assert_eq!(
        c20_result.state(),
        PrivacyPurposeAssessmentState::CompatibleWithReviewedPrivacyPurposeProfile
    );
    assert!(c20_result
        .external_prerequisites()
        .contains(&ExternalPrivacyPrerequisite::SeparatePublicationAdmission));
    assert!(c20_result
        .external_prerequisites()
        .contains(&ExternalPrivacyPrerequisite::SeparatePhysicalExecutionAdmission));
}
