use cos_conformance::{
    authorize_recommendation, bind_general_capability, bind_physical_work,
    bind_plan_to_consumption, bind_plan_to_execution, bind_requirement_to_availability,
    create_source_observation, evaluate_negative, preserve_failure_history, project_to_frs,
    project_to_itc, qualify_output, record_effect, recognize_foreign_evidence, Bindings, Evidence,
    CASES,
};
use serde_json::Value;

const CORPUS: &str =
    include_str!("../../../../docs/integral/d6s-canon-1-golden-vectors.json");

fn evidence_for(profile: &str, at: u64) -> Evidence {
    match profile {
        "current_local" => Evidence::current_local("d6s-canon-1-current", at),
        "stale_local" => Evidence::stale("d6s-canon-1-stale", at),
        "current_foreign_node_b" => {
            Evidence::current_foreign("d6s-canon-1-foreign", "node-b", at)
        }
        other => panic!("unknown corpus evidence profile: {other}"),
    }
}

fn bindings_from_input(input: &Value) -> Bindings {
    let mut bindings = Bindings::default();
    let names = input["bindings"].as_array().expect("input bindings array");
    let mut seen = std::collections::BTreeSet::new();
    for name in names {
        let name = name.as_str().expect("binding name must be string");
        assert!(seen.insert(name), "duplicate binding in fixture: {name}");
        match name {
            "plan_execution" => bind_plan_to_execution(&mut bindings),
            "requirement_availability" => bind_requirement_to_availability(&mut bindings),
            "assignment_observed_work" => cos_conformance::bind_assignment_to_observed_work(&mut bindings),
            "plan_consumption" => bind_plan_to_consumption(&mut bindings),
            "output_qualification" => qualify_output(&mut bindings),
            "foreign_recognition" => recognize_foreign_evidence(&mut bindings),
            "external_dependency_classified" => cos_conformance::classify_external_dependency(&mut bindings),
            "local_capability_claim" => cos_conformance::claim_local_capability(&mut bindings),
            "local_origin_claim" => cos_conformance::claim_local_origin(&mut bindings),
            "itc_projection" => project_to_itc(&mut bindings),
            "recommendation_authorized" => authorize_recommendation(&mut bindings),
            "effect_recorded" => record_effect(&mut bindings),
            "quality_current" => bindings.quality_current = true,
            "general_capability_evidence" => bind_general_capability(&mut bindings),
            "failure_history_preserved" => preserve_failure_history(&mut bindings),
            "source_observation_created" => create_source_observation(&mut bindings),
            "physical_work_binding" => bind_physical_work(&mut bindings),
            "denominator_explicit" => bindings.denominator_explicit = true,
            "frs_projection" => project_to_frs(&mut bindings),
            other => panic!("unknown corpus binding: {other}"),
        }
    }
    bindings
}

fn evaluate_input(test_id: &str, input: &Value, at: u64) -> cos_conformance::Decision {
    let profile = input["evidence_profile"]
        .as_str()
        .expect("evidence_profile must be a string");
    let evidence = evidence_for(profile, at);
    let bindings = bindings_from_input(input);
    evaluate_negative(test_id, &bindings, Some(&evidence), at)
}

#[test]
fn every_canonical_vector_executes_its_explicit_negative_and_positive_inputs() {
    let corpus: Value = serde_json::from_str(CORPUS).expect("pinned corpus must be valid JSON");
    assert_eq!(corpus["schema_version"], "d6s-canon-1-golden-vectors/1");
    assert_eq!(corpus["corpus_id"], "D6S-CANON-1");
    assert_eq!(corpus["status"], "ReferenceModelOnly");
    let vectors = corpus["vectors"].as_array().expect("vectors must be an array");
    assert_eq!(corpus["vector_count"].as_u64(), Some(vectors.len() as u64));
    assert_eq!(vectors.len(), CASES.len());

    let now = 100;
    for (index, vector) in vectors.iter().enumerate() {
        let test_id = vector["test_id"].as_str().expect("test_id must be a string");
        assert_eq!(test_id, CASES[index].test_id, "canonical ordering drift");
        let obligations = vector["formal_obligations"].as_array()
            .expect("each vector must map to formal obligations");
        assert_eq!(obligations.len(), CASES[index].formal_obligations.len());
        for (actual, expected) in obligations.iter().zip(CASES[index].formal_obligations) {
            assert_eq!(actual.as_str(), Some(*expected), "{test_id} formal mapping drift");
        }

        for side in ["negative_input", "positive_input"] {
            assert!(vector[side].is_object(), "{test_id} missing explicit {side}");
            assert!(vector[side]["bindings"].is_array(), "{test_id} missing {side} bindings");
            assert!(vector[side]["evidence_profile"].is_string(), "{test_id} missing {side} evidence profile");
        }

        let positive_bindings = vector["positive_input"]["bindings"]
            .as_array().expect("positive bindings array");
        let must_retain_context: &[&str] = match test_id {
            "COS-N-010" => &["frs_projection", "source_observation_created"],
            "COS-N-011" => &["source_observation_created", "itc_projection"],
            "COS-N-012" => &["itc_projection", "physical_work_binding"],
            _ => &[],
        };
        for required in must_retain_context {
            assert!(
                positive_bindings.iter().any(|binding| binding.as_str() == Some(*required)),
                "{test_id} positive fixture omitted required context binding {required}"
            );
        }

        let negative = evaluate_input(test_id, &vector["negative_input"], now);
        assert_eq!(
            format!("{negative:?}"),
            vector["expected_negative"].as_str().expect("expected_negative string"),
            "{test_id} negative vector changed"
        );

        let positive = evaluate_input(test_id, &vector["positive_input"], now);
        assert_eq!(
            format!("{positive:?}"),
            vector["expected_positive"].as_str().expect("expected_positive string"),
            "{test_id} positive counterpart did not satisfy its explicit bindings"
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
    assert_eq!(report["case_count"].as_u64(), Some(32));
    assert_eq!(report["records"].as_array().map(Vec::len), Some(16));
    for record in report["records"].as_array().expect("report records") {
        assert_eq!(
            record["actual_negative"],
            record["expected_negative"],
            "{} report negative result drift",
            record["test_id"].as_str().unwrap_or("unknown")
        );
        assert_eq!(
            record["actual_positive"],
            record["expected_positive"],
            "{} report positive result drift",
            record["test_id"].as_str().unwrap_or("unknown")
        );
    }
    for record in report["records"].as_array().expect("report records") {
        for field in ["input_evidence_classes", "refinement_subject", "claim_ceiling"] {
            assert!(
                record[field].as_str().is_some_and(|value| !value.trim().is_empty()),
                "report record {} lacks required field {field}",
                record["test_id"].as_str().unwrap_or("unknown")
            );
        }
        assert_eq!(record["refinement_status"], "ReferenceModelOnly");
    }
    assert_eq!(
        report["untested_formal_obligations"].as_array().map(Vec::len),
        Some(0),
        "reference report must disclose no unmapped formal obligation"
    );
}
