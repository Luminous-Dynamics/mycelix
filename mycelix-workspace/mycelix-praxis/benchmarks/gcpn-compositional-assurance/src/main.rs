use gcpn_compositional_assurance::{
    CapReceiptLink, CausalClaim, ClaimDisposition, ClaimRequirement, EvidenceValidity,
    ForecastTiming, PairedComparison, PopulationAccounting, PostProcessingFacts, Scenario,
    ScenarioFacts, Stage, StageEvidence, VerificationAccounting,
};

fn main() {
    let requirements = vec![
        requirement(Stage::Forecast, "forecast-valid", 1),
        requirement(Stage::Observation, "reference-valid", 2),
        requirement(Stage::CapLifecycle, "receipt-valid", 1),
        requirement(Stage::PopulationReach, "reach-valid", 1),
        requirement(Stage::Decision, "decision-valid", 1),
        requirement(Stage::AfterEventReview, "review-valid", 1),
    ];

    let evidence = vec![
        record("f-1", Stage::Forecast, "forecast-valid", "forecast-root"),
        record(
            "o-1",
            Stage::Observation,
            "reference-valid",
            "sensor-root-a",
        ),
        record(
            "o-2",
            Stage::Observation,
            "reference-valid",
            "sensor-root-b",
        ),
        record("c-1", Stage::CapLifecycle, "receipt-valid", "cap-root"),
        record(
            "p-1",
            Stage::PopulationReach,
            "reach-valid",
            "population-root",
        ),
        record("d-1", Stage::Decision, "decision-valid", "decision-root"),
        record(
            "a-1",
            Stage::AfterEventReview,
            "review-valid",
            "review-root",
        ),
    ];

    let scenario = Scenario {
        scenario_id: "composed-happy-path".into(),
        specification_digest: "gcpn-compositional-assurance-v0.1".into(),
        as_of_cutoff: 100,
        requirements,
        evidence,
        facts: ScenarioFacts {
            forecast_timing: Some(ForecastTiming {
                issued_at: 10,
                decision_cutoff: 20,
                event_valid_at: 100,
                minimum_action_lead: 30,
            }),
            cap_receipt: Some(CapReceiptLink {
                current_message_id: "cap-42".into(),
                current_version: "2".into(),
                receipt_message_id: "cap-42".into(),
                receipt_version: "2".into(),
            }),
            population: Some(PopulationAccounting {
                eligible_denominator: Some(1000),
                observed_cases: 900,
                typed_missing_cases: 100,
                unique_reach: 700,
            }),
            verification: Some(VerificationAccounting {
                eligible_cases: 100,
                observed_cases: 90,
                typed_missing_cases: 10,
            }),
            paired_comparison: Some(PairedComparison {
                candidate_case_frame_digest: "frame-1".into(),
                reference_case_frame_digest: "frame-1".into(),
                candidate_source: "source-a".into(),
                reference_source: "source-a".into(),
                candidate_version: "v1".into(),
                reference_version: "v1".into(),
            }),
            causal_claim: Some(CausalClaim {
                claim_made: false,
                identification_method_id: None,
                design_id: None,
                supporting_evidence_ids: vec![],
            }),
            post_processing: Some(PostProcessingFacts {
                intrinsic_skill_claim: true,
                calibration_uses_evaluation_outcomes: false,
                calibration_trained_through: Some(5),
                evaluation_period_start: Some(20),
            }),
        },
    };

    let report = match scenario.audit() {
        Ok(value) => value,
        Err(error) => {
            eprintln!("invalid scenario: {error}");
            std::process::exit(2);
        }
    };

    println!("GCPN compositional assurance v0.1");
    println!("status: deterministic_synthetic_method_demonstration");
    println!("scenario: {}", report.scenario_id);
    println!("specification: {}", report.specification_digest);
    println!("as_of_cutoff: {}", report.as_of_cutoff);
    println!("fingerprint: {}", report.reproducibility_fingerprint);
    println!("stage,claim,disposition,evidence,independent_lineages,violations");
    for claim in &report.claims {
        let violations = claim
            .violations
            .iter()
            .map(|value| value.label())
            .collect::<Vec<_>>()
            .join(",");
        println!(
            "{},{},{},{},{},{}",
            claim.stage.label(),
            claim.claim_id,
            claim.disposition.label(),
            claim.evidence_count,
            claim.independent_supporting_lineages,
            violations
        );
    }
    println!("scenario_violations:");
    for violation in &report.violations {
        println!("- {}", violation.label());
    }
    println!("non_claim: no operational or real-world forecast performance is evaluated");
}

fn requirement(stage: Stage, claim: &str, minimum: usize) -> ClaimRequirement {
    ClaimRequirement {
        stage,
        claim_id: claim.into(),
        subject_digest: "subject-1".into(),
        expected_source: Some("source-a".into()),
        expected_version: Some("v1".into()),
        minimum_independent_supporting_lineages: minimum,
    }
}

fn record(id: &str, stage: Stage, claim: &str, lineage: &str) -> StageEvidence {
    StageEvidence {
        evidence_id: id.into(),
        stage,
        claim_id: claim.into(),
        subject_digest: "subject-1".into(),
        source: "source-a".into(),
        version: "v1".into(),
        event_time: 10,
        recorded_at: 20,
        validity: EvidenceValidity::Current,
        disposition: ClaimDisposition::Supports,
        lineage_root: lineage.into(),
        evaluator_id: format!("evaluator-{id}"),
    }
}
