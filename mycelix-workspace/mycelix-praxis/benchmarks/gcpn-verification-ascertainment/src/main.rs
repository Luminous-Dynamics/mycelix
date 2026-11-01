use gcpn_verification_ascertainment::{
    ComparisonMode, EligibleCase, EvaluationFrame, Metric, MissingReason, ObservedScores,
    OutcomeStatus, ScoreBounds,
};

fn main() {
    let frame = EvaluationFrame {
        target_frame_id: "synthetic-ascertainment-demo".into(),
        score_name: "paired_brier_loss".into(),
        verification_source: "synthetic-known-source".into(),
        verification_version: "fixture-v1".into(),
        comparison: ComparisonMode::Paired {
            candidate_model_id: "candidate-a".into(),
            reference_model_id: "reference-b".into(),
        },
        score_bounds: ScoreBounds {
            lower: 0.0,
            upper: 1.0,
        },
        as_of_cutoff: 100,
        propensity_model_id: Some("declared-fixture-selection-model".into()),
        cases: vec![
            EligibleCase {
                case_id: "case-001".into(),
                independent_system_id: "system-01".into(),
                stratum_id: "well-observed".into(),
                inclusion_probability: Some(0.8),
                outcome: OutcomeStatus::Observed(ObservedScores {
                    candidate_case_id: "case-001".into(),
                    candidate_score: 0.18,
                    reference_case_id: Some("case-001".into()),
                    reference_score: Some(0.25),
                    verification_source: "synthetic-known-source".into(),
                    verification_version: "fixture-v1".into(),
                    recorded_at: 90,
                }),
            },
            EligibleCase {
                case_id: "case-002".into(),
                independent_system_id: "system-01".into(),
                stratum_id: "well-observed".into(),
                inclusion_probability: Some(0.8),
                outcome: OutcomeStatus::Observed(ObservedScores {
                    candidate_case_id: "case-002".into(),
                    candidate_score: 0.30,
                    reference_case_id: Some("case-002".into()),
                    reference_score: Some(0.28),
                    verification_source: "synthetic-known-source".into(),
                    verification_version: "fixture-v1".into(),
                    recorded_at: 92,
                }),
            },
            EligibleCase {
                case_id: "case-003".into(),
                independent_system_id: "system-02".into(),
                stratum_id: "remote-low-coverage".into(),
                inclusion_probability: Some(0.2),
                outcome: OutcomeStatus::Missing {
                    reason: MissingReason::CommunicationUnavailable,
                },
            },
            EligibleCase {
                case_id: "case-004".into(),
                independent_system_id: "system-02".into(),
                stratum_id: "remote-low-coverage".into(),
                inclusion_probability: Some(0.2),
                outcome: OutcomeStatus::Missing {
                    reason: MissingReason::SensorUnavailable,
                },
            },
        ],
    };

    let report = match frame.report() {
        Ok(value) => value,
        Err(error) => {
            eprintln!("invalid evaluation frame: {error}");
            std::process::exit(2);
        }
    };

    println!("GCPN outcome ascertainment qualification v0.1");
    println!("status: synthetic_method_demonstration_not_confirmatory");
    println!("target_frame: {}", report.target_frame_id);
    println!(
        "verification_source: {}@{}",
        report.verification_source, report.verification_version
    );
    println!("as_of_cutoff: {}", report.as_of_cutoff);
    println!("eligible_cases: {}", report.eligible_cases);
    println!("observed_cases: {}", report.observed_cases);
    println!("missing_cases: {}", report.missing_cases);
    println!("verification_coverage: {:.6}", report.coverage_rate);
    println!("independent_systems: {}", report.independent_systems);
    println!(
        "candidate_complete_case_mean: {:?}",
        report.candidate.observed_mean
    );
    println!(
        "candidate_no_assumption_bounds: [{:.6}, {:.6}]",
        report.candidate.population_bounds.lower, report.candidate.population_bounds.upper
    );

    if let Some(difference) = &report.paired_difference {
        println!(
            "candidate_minus_reference_observed_mean: {:?}",
            difference.observed_mean
        );
        println!(
            "candidate_minus_reference_no_assumption_bounds: [{:.6}, {:.6}]",
            difference.population_bounds.lower, difference.population_bounds.upper
        );
    }

    match frame.ipw(Metric::Candidate, 0.05) {
        Ok(estimate) => {
            println!("ipw_propensity_model: {}", estimate.propensity_model_id);
            println!("ipw_ht_mean: {:.6}", estimate.horvitz_thompson_mean);
            println!("ipw_hajek_mean: {:.6}", estimate.hajek_mean);
            println!(
                "ipw_effective_sample_size: {:.6}",
                estimate.effective_sample_size
            );
            println!("ipw_assumption: {}", estimate.identifying_assumption);
        }
        Err(error) => println!("ipw_status: unavailable ({error})"),
    }

    match frame.sensitivity_grid(Metric::Candidate, &[-0.15, 0.0, 0.15]) {
        Ok(points) => {
            println!("sensitivity_delta,assumed_missing_mean,population_mean");
            for point in points {
                println!(
                    "{:.6},{:.6},{:.6}",
                    point.delta, point.assumed_missing_mean, point.population_mean
                );
            }
        }
        Err(error) => println!("sensitivity_status: unavailable ({error})"),
    }

    println!("stratum,eligible,observed,coverage,candidate_lower,candidate_upper");
    for stratum in &report.strata {
        println!(
            "{},{},{},{:.6},{:.6},{:.6}",
            stratum.stratum_id,
            stratum.eligible_cases,
            stratum.observed_cases,
            stratum.coverage_rate,
            stratum.candidate.population_bounds.lower,
            stratum.candidate.population_bounds.upper
        );
    }
    println!("non_claim: no real forecast or operational warning performance is assessed");
}
