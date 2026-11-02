use gcpn_reference_error_spatial_dgp::{
    run_benchmark, DgpTrial, ErrorFamily, ObservationErrorDgp, ParameterKnowledge,
    SpatialFixture,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let fixture = synthetic_fixture();
    let trials: Vec<_> = (0..24)
        .map(|index| DgpTrial {
            seed: 10_000 + index,
            event_block_id: format!("event-block-{}", index / 3),
        })
        .collect();

    let dgp = ObservationErrorDgp {
        family: ErrorFamily::SpatiallyCorrelated,
        severity: 0.3,
        correlation_radius: 2,
        parameter_knowledge: ParameterKnowledge::UnknownRange {
            lower: 0.05,
            upper: 0.6,
        },
    };

    let report = run_benchmark(&fixture, &dgp, &trials, &[1, 3, 5, 7], 128)?;

    println!("# Synthetic latent-truth oracle and observed-reference robustness only.");
    println!("# Forecast fields are fixed; observation error changes between trials.");
    println!("{}", report.summary_line());
    print!("{}", report.to_csv());
    println!("# no_real_forecast_data=true");
    println!("# unknown_error_parameters_are_sensitivity_only=true");

    Ok(())
}

fn synthetic_fixture() -> SpatialFixture {
    let rows = 18;
    let columns = 18;
    let mut latent = vec![0.0; rows * columns];

    for row in 0..rows {
        for column in 0..columns {
            let first = (3..=7).contains(&row) && (3..=7).contains(&column);
            let second = (10..=14).contains(&row) && (11..=15).contains(&column);
            let third = (7..=9).contains(&row) && (12..=14).contains(&column);
            latent[row * columns + column] = if first || second || third { 2.0 } else { 0.0 };
        }
    }

    let mut reference = vec![0.0; rows * columns];
    for row in 0..rows {
        for column in 0..columns {
            let source_column = (column + columns - 2) % columns;
            reference[row * columns + column] = latent[row * columns + source_column];
        }
    }

    SpatialFixture {
        case_id: "synthetic-heavy-rain-event".into(),
        rows,
        columns,
        threshold: 1.0,
        unit: "mm/h".into(),
        candidate_forecast: latent.clone(),
        reference_forecast: reference,
        latent_truth: latent,
        cell_weights: vec![1.0; rows * columns],
        valid_mask: vec![true; rows * columns],
    }
}
