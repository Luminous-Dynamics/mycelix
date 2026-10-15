use cos_conformance::{
    bind_general_capability, bind_physical_work,
    bind_plan_to_consumption, bind_plan_to_execution, bind_requirement_to_availability,
    authorize_recommendation, create_source_observation, evaluate_negative,
    preserve_failure_history, project_to_itc, qualify_output,
    record_effect, recognize_foreign_evidence, Bindings, Decision, Evidence, CASES,
};
use serde_json::Value;

const CORPUS: &str =
    include_str!("../../../../docs/integral/d6s-canon-1-golden-vectors.json");

fn positive_bindings(test_id: &str) -> Bindings {
    let mut bindings = Bindings::default();
    match test_id {
        "COS-N-001" => bind_plan_to_execution(&mut bindings),
        "COS-N-002" => bind_plan_to_consumption(&mut bindings),
        "COS-N-003" | "COS-N-004" => bind_requirement_to_availability(&mut bindings),
        "COS-N-005" => bindings.quality_current = true,
        "COS-N-006" | "COS-N-015" => qualify_output(&mut bindings),
        "COS-N-007" => bind_general_capability(&mut bindings),
        "COS-N-008" => preserve_failure_history(&mut bindings),
        "COS-N-009" | "COS-N-013" => recognize_foreign_evidence(&mut bindings),
        "COS-N-010" => create_source_observation(&mut bindings),
        "COS-N-011" => project_to_itc(&mut bindings),
        "COS-N-012" => bind_physical_work(&mut bindings),
        "COS-N-014" => {
            authorize_recommendation(&mut bindings);
            record_effect(&mut bindings);
        }
        "COS-N-016" => bindings.denominator_explicit = true,
        _ => panic!("unexpected canonical test ID: {test_id}"),
    }
    bindings
}

#[test]
fn every_canonical_vector_matches_the_executable_reference_model() {
    let corpus: Value = serde_json::from_str(CORPUS).expect("pinned corpus must be valid JSON");
    assert_eq!(corpus["schema_version"], "d6s-canon-1-golden-vectors/1");
    assert_eq!(corpus["corpus_id"], "D6S-CANON-1");
    assert_eq!(corpus["status"], "ReferenceModelOnly");
    let vectors = corpus["vectors"].as_array().expect("vectors must be an array");
    assert_eq!(corpus["vector_count"].as_u64(), Some(vectors.len() as u64));
    assert_eq!(vectors.len(), CASES.len());

    let now = 100;
    let current = Evidence::current_local("d6s-canon-1-current", now);
    let stale = Evidence::stale("d6s-canon-1-stale", now);
    let foreign = Evidence::current_foreign("d6s-canon-1-foreign", "node-b", now);
    for (index, vector) in vectors.iter().enumerate() {
        let test_id = vector["test_id"].as_str().expect("test_id must be a string");
        assert_eq!(test_id, CASES[index].test_id, "canonical ordering drift");
        let obligations = vector["formal_obligations"].as_array()
            .expect("each vector must map to formal obligations");
        assert_eq!(obligations.len(), CASES[index].formal_obligations.len());
        for (actual, expected) in obligations.iter().zip(CASES[index].formal_obligations) {
            assert_eq!(actual.as_str(), Some(*expected), "{test_id} formal mapping drift");
        }

        let negative_bindings = Bindings::default();
        let negative_evidence = if test_id == "COS-N-005" { &stale } else { &current };
        let negative = evaluate_negative(test_id, &negative_bindings, Some(negative_evidence), now);
        assert_eq!(
            format!("{negative:?}"),
            vector["expected_negative"].as_str().expect("expected_negative string"),
            "{test_id} negative vector changed"
        );

        let positive_evidence = if matches!(test_id, "COS-N-009" | "COS-N-013") {
            &foreign
        } else {
            &current
        };
        let positive = evaluate_negative(
            test_id,
            &positive_bindings(test_id),
            Some(positive_evidence),
            now,
        );
        assert_eq!(
            format!("{positive:?}"),
            vector["expected_positive"].as_str().expect("expected_positive string"),
            "{test_id} positive counterpart did not satisfy explicit binding"
        );
    }
}

#[test]
fn canonical_vectors_cover_each_formal_obligation() {
    let corpus: Value = serde_json::from_str(CORPUS).expect("corpus JSON");
    let mut observed = std::collections::BTreeSet::new();
    for vector in corpus["vectors"].as_array().expect("vectors") {
        for obligation in vector["formal_obligations"].as_array().expect("obligations") {
            observed.insert(obligation.as_str().expect("obligation string").to_owned());
        }
    }
    let expected: std::collections::BTreeSet<String> =
        cos_conformance::FORMAL_OBLIGATIONS.iter().map(|x| (*x).to_owned()).collect();
    assert_eq!(observed, expected, "every COS-FV obligation must have corpus coverage");
    let report: Value = serde_json::from_str(&cos_conformance::conformance_report_json())
        .expect("reference report JSON");
    assert_eq!(
        report["untested_formal_obligations"].as_array().map(Vec::len),
        Some(0),
        "reference report must disclose no unmapped formal obligation"
    );
}
