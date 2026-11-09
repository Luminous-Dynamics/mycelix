use emsr_causal_intervention_benchmark::{
    CandidateDisposition, CandidateReport, HiddenCausalPath, HiddenTruth, InterventionPlan,
    PublicInput, ResponseState, TimeMapping, Trial, TrialArm, TrialObservation,
    TRUTH_CONTRACT, evaluate_public_bytes, hidden_truth_commitment_sha256, score_candidate,
};

fn public_input(active_degraded: &[&str], sham_degraded: &[&str]) -> PublicInput {
    let expected_source_ids = vec!["endpoint-a".to_owned(), "endpoint-b".to_owned()];
    let trials = vec![
        trial("active-1", TrialArm::Active, 0, 0),
        trial("sham-1", TrialArm::Sham, 1, 200),
        trial("active-2", TrialArm::Active, 2, 400),
        trial("sham-2", TrialArm::Sham, 3, 600),
    ];
    let mut sequence_by_source = std::collections::BTreeMap::new();
    let mut observations = Vec::new();
    for trial in &trials {
        for source in &expected_source_ids {
            let sequence = sequence_by_source.entry(source.clone()).or_insert(0_u64);
            *sequence += 1;
            let is_degraded = match trial.arm {
                TrialArm::Active => active_degraded.contains(&source.as_str()),
                TrialArm::Sham => sham_degraded.contains(&source.as_str()),
            };
            observations.push(TrialObservation {
                trial_id: trial.trial_id.clone(),
                source_id: source.clone(),
                event_time_tai_ns: trial.observation_start_tai_ns,
                received_time_tai_ns: trial.observation_start_tai_ns + 25,
                sequence: *sequence,
                time_mapping: TimeMapping::Qualified,
                state: if is_degraded {
                    ResponseState::Degraded
                } else {
                    ResponseState::Nominal
                },
            });
        }
    }
    PublicInput {
        contract: emsr_causal_intervention_benchmark::INPUT_CONTRACT.to_owned(),
        scenario_id: "scenario-controlled-001".to_owned(),
        decision_scope: "lab-clock-domain-a".to_owned(),
        expected_source_ids,
        intervention: InterventionPlan {
            intervention_id: "inject-grandmaster-phase-step".to_owned(),
            target_id: "grandmaster-a".to_owned(),
            domain: "clock".to_owned(),
            intervention_kind: "controlled_phase_disturbance".to_owned(),
            minimum_latency_ns: 10,
            maximum_latency_ns: 100,
            minimum_active_trials: 2,
            minimum_sham_trials: 2,
            response_threshold_milli: 750,
            maximum_sham_response_rate_milli: 100,
        },
        trials,
        observations,
    }
}

fn trial(id: &str, arm: TrialArm, order: u64, injection: u64) -> Trial {
    Trial {
        trial_id: id.to_owned(),
        arm,
        randomized_order: order,
        injection_time_tai_ns: injection,
        observation_start_tai_ns: injection + 10,
        observation_end_tai_ns: injection + 90,
        randomization_receipt_sha256: "a".repeat(64),
    }
}

fn hidden_truth(common_sources: &[&str]) -> HiddenTruth {
    let mut truth = HiddenTruth {
        contract: TRUTH_CONTRACT.to_owned(),
        scenario_id: "scenario-controlled-001".to_owned(),
        causal_paths: common_sources
            .iter()
            .map(|source| HiddenCausalPath {
                intervention_target_id: "grandmaster-a".to_owned(),
                source_id: (*source).to_owned(),
                domain: "clock".to_owned(),
                path_node_ids: vec![
                    "grandmaster-a".to_owned(),
                    "shared-clock-root".to_owned(),
                    (*source).to_owned(),
                ],
                minimum_latency_ns: 15,
                maximum_latency_ns: 70,
            })
            .collect(),
        truth_commitment_sha256: String::new(),
    };
    truth.truth_commitment_sha256 =
        hidden_truth_commitment_sha256(&truth).expect("truth commitment");
    truth
}

fn report_bytes(input: &PublicInput) -> (Vec<u8>, CandidateReport) {
    let public_bytes = serde_json::to_vec(input).expect("public input JSON");
    let report = evaluate_public_bytes(&public_bytes);
    let bytes = serde_json::to_vec(&report).expect("candidate report JSON");
    (bytes, report)
}

fn score(input: &PublicInput, truth: &HiddenTruth) -> emsr_causal_intervention_benchmark::EvaluationScore {
    let public_bytes = serde_json::to_vec(input).expect("public input JSON");
    let (report_bytes, _) = report_bytes(input);
    let truth_bytes = serde_json::to_vec(truth).expect("hidden truth JSON");
    score_candidate(&public_bytes, &truth_bytes, &report_bytes)
}

#[test]
fn active_intervention_effect_in_two_endpoints_supports_scoped_common_cause() {
    let input = public_input(&["endpoint-a", "endpoint-b"], &[]);
    let public_bytes = serde_json::to_vec(&input).expect("public input");
    let report = evaluate_public_bytes(&public_bytes);
    assert_eq!(
        report.disposition,
        CandidateDisposition::CommonCauseSupportedWithinInterventionScope
    );
    assert_eq!(
        report.responsive_source_ids,
        vec!["endpoint-a".to_owned(), "endpoint-b".to_owned()]
    );

    let evaluated = score(&input, &hidden_truth(&["endpoint-a", "endpoint-b"]));
    assert!(evaluated.ground_truth_common_cause);
    assert!(evaluated.true_positive);
    assert!(!evaluated.false_independence_claim);
}

#[test]
fn no_declared_causal_paths_and_no_active_effect_is_a_scoped_true_negative() {
    let input = public_input(&[], &[]);
    let report = evaluate_public_bytes(&serde_json::to_vec(&input).unwrap());
    assert_eq!(
        report.disposition,
        CandidateDisposition::NoCommonCauseDetectedWithinTestedEnvelope
    );
    let evaluated = score(&input, &hidden_truth(&[]));
    assert!(!evaluated.ground_truth_common_cause);
    assert!(evaluated.true_negative);
    assert!(!evaluated.false_independence_claim);
}

#[test]
fn sham_response_is_a_nuisance_control_and_forces_abstention() {
    let input = public_input(
        &["endpoint-a", "endpoint-b"],
        &["endpoint-a", "endpoint-b"],
    );
    let report = evaluate_public_bytes(&serde_json::to_vec(&input).unwrap());
    assert_eq!(
        report.disposition,
        CandidateDisposition::IndeterminateInterventionCoverage
    );
    assert!(
        report.reasons.iter().any(|reason| {
            reason == "sham_trial_response_indicates_nuisance_or_noncausal_correlation"
        })
    );
    let evaluated = score(&input, &hidden_truth(&["endpoint-a", "endpoint-b"]));
    assert!(evaluated.abstained);
    assert!(!evaluated.false_independence_claim);
}

#[test]
fn missing_observation_never_becomes_a_clean_no_common_cause_result() {
    let mut input = public_input(&[], &[]);
    input.observations.pop();
    let report = evaluate_public_bytes(&serde_json::to_vec(&input).unwrap());
    assert_eq!(
        report.disposition,
        CandidateDisposition::IndeterminateInterventionCoverage
    );
}

#[test]
fn unknown_clock_mapping_forces_abstention() {
    let mut input = public_input(&["endpoint-a", "endpoint-b"], &[]);
    input.observations[0].time_mapping = TimeMapping::Unknown;
    let report = evaluate_public_bytes(&serde_json::to_vec(&input).unwrap());
    assert_eq!(
        report.disposition,
        CandidateDisposition::IndeterminateInterventionCoverage
    );
}

#[test]
fn invalid_clock_mapping_is_not_misreported_as_low_coverage() {
    let mut input = public_input(&[], &[]);
    input.observations[0].time_mapping = TimeMapping::Invalid;
    let report = evaluate_public_bytes(&serde_json::to_vec(&input).unwrap());
    assert_eq!(
        report.disposition,
        CandidateDisposition::ObservationOrClockMappingInvalid
    );
}

#[test]
fn receive_delay_does_not_rewrite_event_time_or_change_the_decision() {
    let mut input = public_input(&["endpoint-a", "endpoint-b"], &[]);
    let original = evaluate_public_bytes(&serde_json::to_vec(&input).unwrap());
    for observation in &mut input.observations {
        observation.received_time_tai_ns += 10_000_000;
    }
    let delayed = evaluate_public_bytes(&serde_json::to_vec(&input).unwrap());
    assert_eq!(original.disposition, delayed.disposition);
    assert_eq!(original.responsive_source_ids, delayed.responsive_source_ids);
    assert_ne!(original.input_sha256, delayed.input_sha256);
}

#[test]
fn hidden_common_cause_missed_by_candidate_is_counted_as_false_independence_claim() {
    let input = public_input(&[], &[]);
    let evaluated = score(&input, &hidden_truth(&["endpoint-a", "endpoint-b"]));
    assert!(evaluated.ground_truth_common_cause);
    assert!(evaluated.false_negative);
    assert!(evaluated.false_independence_claim);
}

#[test]
fn a_synchronous_unrelated_fault_can_create_a_false_positive_and_is_counted() {
    let input = public_input(&["endpoint-a", "endpoint-b"], &[]);
    let evaluated = score(&input, &hidden_truth(&["endpoint-a"]));
    assert!(!evaluated.ground_truth_common_cause);
    assert!(evaluated.false_positive);
    assert!(!evaluated.true_positive);
}

#[test]
fn changed_public_bytes_cannot_reuse_an_old_candidate_report() {
    let input = public_input(&["endpoint-a", "endpoint-b"], &[]);
    let (report_bytes, _) = report_bytes(&input);
    let mut changed = input.clone();
    changed.decision_scope = "different-scope".to_owned();
    let public_bytes = serde_json::to_vec(&changed).unwrap();
    let truth_bytes = serde_json::to_vec(&hidden_truth(&["endpoint-a", "endpoint-b"])).unwrap();
    let evaluated = score_candidate(&public_bytes, &truth_bytes, &report_bytes);
    assert!(
        evaluated
            .limitations
            .iter()
            .any(|reason| reason == "candidate_or_truth_binding_invalid")
    );
}

#[test]
fn modified_hidden_topology_without_updated_commitment_is_rejected() {
    let input = public_input(&[], &[]);
    let (report, _) = report_bytes(&input);
    let mut truth = hidden_truth(&[]);
    truth.causal_paths.push(HiddenCausalPath {
        intervention_target_id: "grandmaster-a".to_owned(),
        source_id: "endpoint-a".to_owned(),
        domain: "clock".to_owned(),
        path_node_ids: vec!["grandmaster-a".to_owned(), "endpoint-a".to_owned()],
        minimum_latency_ns: 15,
        maximum_latency_ns: 70,
    });
    let public_bytes = serde_json::to_vec(&input).unwrap();
    let truth_bytes = serde_json::to_vec(&truth).unwrap();
    let evaluated = score_candidate(&public_bytes, &truth_bytes, &report);
    assert!(
        evaluated
            .limitations
            .iter()
            .any(|reason| reason == "candidate_or_truth_binding_invalid")
    );
}

#[test]
fn duplicate_trial_source_observation_is_invalid() {
    let mut input = public_input(&[], &[]);
    input.observations.push(input.observations[0].clone());
    let report = evaluate_public_bytes(&serde_json::to_vec(&input).unwrap());
    assert_eq!(
        report.disposition,
        CandidateDisposition::ObservationOrClockMappingInvalid
    );
}
