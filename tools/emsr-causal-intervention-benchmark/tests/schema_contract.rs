use emsr_causal_intervention_benchmark::{
    EvaluationScore, HiddenTruth, PublicInput, ResponseState, TimeMapping, TrialObservation, evaluate_public_bytes,
    hidden_truth_commitment_sha256, score_candidate,
};
use jsonschema::draft202012;
use serde_json::Value;

const INPUT_SCHEMA: &str =
    include_str!("../../../docs/assurance/causal-intervention-public-input-0.1.schema.json");
const REPORT_SCHEMA: &str =
    include_str!("../../../docs/assurance/causal-intervention-candidate-report-0.1.schema.json");
const TRUTH_SCHEMA: &str =
    include_str!("../../../docs/assurance/causal-intervention-hidden-truth-0.1.schema.json");
const SCORE_SCHEMA: &str =
    include_str!("../../../docs/assurance/causal-intervention-evaluation-score-0.1.schema.json");

fn validate(schema_text: &str, value: &Value) {
    let schema: Value = serde_json::from_str(schema_text).expect("schema JSON");
    assert!(draft202012::meta::is_valid(&schema));
    let validator = draft202012::new(&schema).expect("Draft 2020-12 schema");
    assert!(
        validator.is_valid(value),
        "schema rejected value: {value:#}"
    );
}

#[test]
fn input_report_truth_and_score_match_closed_schemas() {
    let mut input: PublicInput = serde_json::from_value(serde_json::json!({
        "contract":"emsr.causal-intervention-public-input/0.1",
        "scenario_id":"schema-scenario-001",
        "decision_scope":"test-lab",
        "expected_source_ids":["endpoint-a","endpoint-b"],
        "negative_control_source_ids":["collector-control"],
        "intervention":{
            "intervention_id":"inject-clock-fault",
            "target_id":"grandmaster-a",
            "domain":"clock",
            "intervention_kind":"phase-step",
            "minimum_latency_ns":10,
            "maximum_latency_ns":100,
            "minimum_active_trials":2,
            "minimum_sham_trials":2,
            "response_threshold_milli":750,
            "maximum_sham_response_rate_milli":100
        },
        "trials":[
            {"trial_id":"a1","arm":"active","randomized_order":0,"injection_time_tai_ns":0,"observation_start_tai_ns":10,"observation_end_tai_ns":90,"randomization_receipt_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"},
            {"trial_id":"s1","arm":"sham","randomized_order":1,"injection_time_tai_ns":200,"observation_start_tai_ns":210,"observation_end_tai_ns":290,"randomization_receipt_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"},
            {"trial_id":"a2","arm":"active","randomized_order":2,"injection_time_tai_ns":400,"observation_start_tai_ns":410,"observation_end_tai_ns":490,"randomization_receipt_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"},
            {"trial_id":"s2","arm":"sham","randomized_order":3,"injection_time_tai_ns":600,"observation_start_tai_ns":610,"observation_end_tai_ns":690,"randomization_receipt_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"}
        ],
        "observations":[
            {"trial_id":"a1","source_id":"endpoint-a","event_time_tai_ns":10,"received_time_tai_ns":15,"sequence":1,"time_mapping":"qualified","state":"degraded"},
            {"trial_id":"a1","source_id":"endpoint-b","event_time_tai_ns":10,"received_time_tai_ns":15,"sequence":1,"time_mapping":"qualified","state":"degraded"},
            {"trial_id":"s1","source_id":"endpoint-a","event_time_tai_ns":210,"received_time_tai_ns":215,"sequence":2,"time_mapping":"qualified","state":"nominal"},
            {"trial_id":"s1","source_id":"endpoint-b","event_time_tai_ns":210,"received_time_tai_ns":215,"sequence":2,"time_mapping":"qualified","state":"nominal"},
            {"trial_id":"a2","source_id":"endpoint-a","event_time_tai_ns":410,"received_time_tai_ns":415,"sequence":3,"time_mapping":"qualified","state":"degraded"},
            {"trial_id":"a2","source_id":"endpoint-b","event_time_tai_ns":410,"received_time_tai_ns":415,"sequence":3,"time_mapping":"qualified","state":"degraded"},
            {"trial_id":"s2","source_id":"endpoint-a","event_time_tai_ns":610,"received_time_tai_ns":615,"sequence":4,"time_mapping":"qualified","state":"nominal"},
            {"trial_id":"s2","source_id":"endpoint-b","event_time_tai_ns":610,"received_time_tai_ns":615,"sequence":4,"time_mapping":"qualified","state":"nominal"}
        ]
    })).expect("public input fixture");
    input.scenario_id = "scenario-controlled-001".to_owned();
    for (index, trial) in input.trials.iter().enumerate() {
        input.observations.push(TrialObservation {
            trial_id: trial.trial_id.clone(),
            source_id: "collector-control".to_owned(),
            event_time_tai_ns: trial.observation_start_tai_ns,
            received_time_tai_ns: trial.observation_start_tai_ns + 5,
            sequence: (index + 1) as u64,
            time_mapping: TimeMapping::Qualified,
            state: ResponseState::Nominal,
        });
    }

    let public_bytes = serde_json::to_vec(&input).expect("public input bytes");
    validate(INPUT_SCHEMA, &serde_json::to_value(&input).unwrap());
    let report = evaluate_public_bytes(&public_bytes);
    validate(REPORT_SCHEMA, &serde_json::to_value(&report).unwrap());

    let mut truth: HiddenTruth = serde_json::from_value(serde_json::json!({
        "contract":"emsr.causal-intervention-hidden-truth/0.1",
        "scenario_id":"scenario-controlled-001",
        "causal_paths":[
            {"intervention_target_id":"grandmaster-a","source_id":"endpoint-a","domain":"clock","path_node_ids":["grandmaster-a","clock-root","endpoint-a"],"minimum_latency_ns":15,"maximum_latency_ns":70},
            {"intervention_target_id":"grandmaster-a","source_id":"endpoint-b","domain":"clock","path_node_ids":["grandmaster-a","clock-root","endpoint-b"],"minimum_latency_ns":15,"maximum_latency_ns":70}
        ],
        "truth_commitment_sha256":"0000000000000000000000000000000000000000000000000000000000000000"
    })).expect("hidden truth");
    truth.truth_commitment_sha256 = hidden_truth_commitment_sha256(&truth).unwrap();
    validate(TRUTH_SCHEMA, &serde_json::to_value(&truth).unwrap());

    let truth_bytes = serde_json::to_vec(&truth).unwrap();
    let report_bytes = serde_json::to_vec(&report).unwrap();
    let score: EvaluationScore = serde_json::from_value(
        serde_json::to_value(score_candidate(&public_bytes, &truth_bytes, &report_bytes)).unwrap(),
    )
    .unwrap();
    validate(SCORE_SCHEMA, &serde_json::to_value(&score).unwrap());
    assert!(score.true_positive);
}
