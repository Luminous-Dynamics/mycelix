use gcpn_probabilistic_fss::{
    BlockBootstrapConfig, BoundaryPolicy, FssBlockCase, FssCase, FssConfig, MissingPolicy,
    RandomBaselineConfig, ThresholdOperator, block_bootstrap, evaluate, random_reference,
};

fn optional_values(events: &[bool]) -> Vec<Option<f64>> {
    events
        .iter()
        .map(|event| Some(if *event { 12.0 } else { 0.0 }))
        .collect()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let observed = [
        false, false, false, false, false, false, true, true, false, false, false, false, true,
        false, false, false, false, false, false, false, false, false, false, false, false,
    ];
    let member_a = [
        false, false, false, false, false, false, true, false, false, false, false, false, true,
        true, false, false, false, false, false, false, false, false, false, false, false,
    ];
    let member_b = [
        false, false, false, false, false, false, true, true, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false, false, false,
    ];
    let member_c = [
        false, false, false, false, false, false, false, true, false, false, false, false, true,
        false, false, false, false, false, false, false, false, false, false, false, false,
    ];

    let case = FssCase {
        case_id: "synthetic-flood-threshold-case".into(),
        rows: 5,
        columns: 5,
        forecast_members: vec![
            optional_values(&member_a),
            optional_values(&member_b),
            optional_values(&member_c),
        ],
        observation: optional_values(&observed),
        cell_weights: vec![1.0; 25],
        valid_mask: vec![true; 25],
    };
    let config = FssConfig {
        threshold: 1.0,
        threshold_operator: ThresholdOperator::GreaterThanOrEqual,
        unit: "mm_per_hour".into(),
        window_width_cells: 3,
        boundary_policy: BoundaryPolicy::TruncateAtDomain,
        missing_policy: MissingPolicy::RejectCase,
    };

    let score = evaluate(&case, &config)?;
    let random = random_reference(
        &case,
        &config,
        &RandomBaselineConfig {
            draws: 999,
            seed: 20261102,
        },
    )?;
    let mut block_case_a = case.clone();
    block_case_a.case_id = "synthetic-event-case-a".into();
    let mut block_case_b = case.clone();
    block_case_b.case_id = "synthetic-event-case-b".into();
    let mut block_case_c = case.clone();
    block_case_c.case_id = "synthetic-event-case-c".into();
    let block_samples = vec![
        FssBlockCase {
            event_block_id: "event-block-a".into(),
            case: block_case_a,
        },
        FssBlockCase {
            event_block_id: "event-block-b".into(),
            case: block_case_b,
        },
        FssBlockCase {
            event_block_id: "event-block-c".into(),
            case: block_case_c,
        },
    ];
    let bootstrap = block_bootstrap(
        &block_samples,
        &config,
        &BlockBootstrapConfig {
            replicates: 199,
            seed: 20261103,
        },
    )?;

    println!("# Synthetic pFSS demonstration; no real forecast data or scores.");
    println!("case_id={}", score.case_id);
    println!("status={}", score.status.label());
    println!("threshold={} {}", score.threshold, score.unit);
    println!("threshold_operator={}", score.threshold_operator.label());
    println!("window_width_cells={}", score.window_width_cells);
    println!("ensemble_members={}", score.ensemble_members);
    println!("usable_grid_cells={}", score.usable_grid_cells);
    println!("centre_count={}", score.centre_count);
    println!(
        "observation_event_frequency={:.8}",
        score.observation_event_frequency
    );
    println!("fractions_brier_score={:.8}", score.fractions_brier_score);
    println!(
        "worst_fractions_brier_score={:.8}",
        score.worst_fractions_brier_score
    );
    println!("p_fss={:.8}", score.p_fss.unwrap_or(f64::NAN));
    println!("random_null_draws={}", random.draws);
    println!("random_null_seed={}", random.seed);
    println!("random_null_mean={:.8}", random.null_mean);
    println!("random_null_p95={:.8}", random.null_p95);
    println!(
        "random_null_upper_tail_p_value={:.8}",
        random.upper_tail_p_value
    );
    println!("skill_reference=empirical_independent_bernoulli_null");
    println!(
        "block_bootstrap_blocks={}",
        bootstrap.independent_event_blocks
    );
    println!(
        "block_bootstrap_replicates={}",
        bootstrap.requested_replicates
    );
    println!(
        "block_bootstrap_qualified={}",
        bootstrap.qualified_replicates
    );
    println!(
        "block_bootstrap_degenerate={}",
        bootstrap.degenerate_replicates
    );
    println!(
        "block_bootstrap_pooling={}",
        bootstrap.pooling_policy.label()
    );
    println!(
        "block_bootstrap_ci95={:.8},{:.8}",
        bootstrap.lower_95_percent, bootstrap.upper_95_percent
    );
    println!("score_calibration=not_assessed");
    println!("crps_brier_energy_variogram=not_computed");

    Ok(())
}
