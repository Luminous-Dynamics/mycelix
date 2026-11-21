use space_defensive_importance_sampling::{
    CollisionObservation, DefensiveGaussianMixture, EvaluationEvidence, GaussianComponent,
    MultifidelityResidualObservation, MultifidelitySurrogateEvidence, WhiteningTransform,
    estimate_collision_probability, estimate_multifidelity_residual_probability,
    exact_standard_normal_2d_disk_probability,
};
use std::process;

const SAMPLES_PER_REPLICATE: usize = 8_192;
const REPLICATES: usize = 32;
const DELTA: f64 = 0.05;
const DEFENSIVE_EPSILON: f64 = 0.05;

struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn uniform_open(&mut self) -> f64 {
        // Midpoint mapping avoids exact zero and one.
        ((self.next_u64() >> 11) as f64 + 0.5) / 9_007_199_254_740_992.0
    }

    fn normal_pair(&mut self) -> [f64; 2] {
        let radius = (-2.0 * self.uniform_open().ln()).sqrt();
        let angle = std::f64::consts::TAU * self.uniform_open();
        [radius * angle.cos(), radius * angle.sin()]
    }
}

fn whitening() -> WhiteningTransform {
    WhiteningTransform {
        id: "analytic-standard-normal-whitening-v1".into(),
        revision: "fixture:identity-v1".into(),
        state_frame_id: "dimensionless-standard-normal-v1".into(),
        state_units_id: "dimensionless-v1".into(),
        state_layout_id: "two-dimensional-static-state-v1".into(),
        state_epoch_id: "static-fixture-epoch-v1".into(),
        time_scale_id: "static-no-time-scale-v1".into(),
        origin_id: "disk-center-v1".into(),
        mean: vec![0.0, 0.0],
        cholesky: vec![vec![1.0, 0.0], vec![0.0, 1.0]],
    }
}

fn proposal(id: &str, epsilon: f64, adaptive_sigma: f64) -> DefensiveGaussianMixture {
    DefensiveGaussianMixture {
        id: id.into(),
        epsilon,
        dimension: 2,
        components: vec![GaussianComponent {
            id: "single-adaptive-gaussian".into(),
            mixture_weight: 1.0,
            mean: vec![0.0, 0.0],
            cholesky: vec![vec![adaptive_sigma, 0.0], vec![0.0, adaptive_sigma]],
        }],
    }
}

fn run_replicate(
    proposal: &DefensiveGaussianMixture,
    radius: f64,
    seed: u64,
    replicate: usize,
    method_id: &str,
) -> Result<(f64, f64), Box<dyn std::error::Error>> {
    let mut rng = SplitMix64::new(seed);
    let mut observations = Vec::with_capacity(SAMPLES_PER_REPLICATE);

    for _ in 0..SAMPLES_PER_REPLICATE {
        let latent = rng.normal_pair();
        let sample =
            proposal.sample_from_latents(rng.uniform_open(), rng.uniform_open(), &latent)?;
        let squared_radius = sample.point[0] * sample.point[0] + sample.point[1] * sample.point[1];
        observations.push(CollisionObservation {
            point: sample.point.clone(),
            physical_state: sample.point,
            collision: squared_radius <= radius * radius,
        });
    }

    let evaluation_id = format!("{method_id}-radius-{radius:.6}-replicate-{replicate}");
    let evidence = EvaluationEvidence {
        proposal_id: proposal.id.clone(),
        whitening_transform_id: "analytic-standard-normal-whitening-v1".into(),
        whitening_transform_revision: "fixture:identity-v1".into(),
        state_frame_id: "dimensionless-standard-normal-v1".into(),
        state_units_id: "dimensionless-v1".into(),
        state_layout_id: "two-dimensional-static-state-v1".into(),
        state_epoch_id: "static-fixture-epoch-v1".into(),
        time_scale_id: "static-no-time-scale-v1".into(),
        origin_id: "disk-center-v1".into(),
        proposal_training_id: "analytic-fixture-proposal-training-v1".into(),
        evaluation_sample_id: evaluation_id.clone(),
        sampling_design_id: "seeded-splitmix-box-muller-synthetic-regression-v1".into(),
        sampling_evidence_ref: "fixture://prng-regression-only/not-production-rng-certification"
            .into(),
        sample_order_digest: format!("fixture-order-seed-{seed:016x}"),
        event_oracle_id: "exact-centered-standard-normal-disk".into(),
        event_oracle_revision: "closed-form-1-exp-minus-r2-over-2-v1".into(),
        event_oracle_is_high_fidelity_continuous_time: false,
        event_oracle_is_analytic_reference_fixture: true,
        proposal_frozen_before_evaluation: true,
        evaluation_draws_are_iid_from_full_mixture: true,
    };

    let report =
        estimate_collision_probability(proposal, &whitening(), &observations, DELTA, &evidence)?;
    Ok((report.estimate, report.upper_confidence_bound))
}

struct MultifidelityReplicate {
    estimate: f64,
    lower: f64,
    upper: f64,
    residual_variance: f64,
    direct_variance: f64,
    variance_reduction_ratio: Option<f64>,
    false_positive_count: usize,
    false_negative_count: usize,
}

fn run_multifidelity_replicate(
    proposal: &DefensiveGaussianMixture,
    high_fidelity_radius: f64,
    seed: u64,
    replicate: usize,
) -> Result<MultifidelityReplicate, Box<dyn std::error::Error>> {
    let surrogate_radius = high_fidelity_radius * 1.1;
    let mut rng = SplitMix64::new(seed);
    let mut observations = Vec::with_capacity(SAMPLES_PER_REPLICATE);

    for sample_index in 0..SAMPLES_PER_REPLICATE {
        let latent = rng.normal_pair();
        let sample =
            proposal.sample_from_latents(rng.uniform_open(), rng.uniform_open(), &latent)?;
        let squared_radius = sample.point[0] * sample.point[0] + sample.point[1] * sample.point[1];
        observations.push(MultifidelityResidualObservation {
            sample_id: format!("mf-row-{replicate:04}-{sample_index:08}"),
            point: sample.point.clone(),
            physical_state: sample.point,
            surrogate_collision: squared_radius <= surrogate_radius * surrogate_radius,
            high_fidelity_collision: squared_radius <= high_fidelity_radius * high_fidelity_radius,
        });
    }

    let evaluation_id = format!("multifidelity-evaluation-{high_fidelity_radius:.6}-{replicate}");
    let sample_order_digest = format!("fixture-order-seed-{seed:016x}");
    let evaluation = EvaluationEvidence {
        proposal_id: proposal.id.clone(),
        whitening_transform_id: "analytic-standard-normal-whitening-v1".into(),
        whitening_transform_revision: "fixture:identity-v1".into(),
        state_frame_id: "dimensionless-standard-normal-v1".into(),
        state_units_id: "dimensionless-v1".into(),
        state_layout_id: "two-dimensional-static-state-v1".into(),
        state_epoch_id: "static-fixture-epoch-v1".into(),
        time_scale_id: "static-no-time-scale-v1".into(),
        origin_id: "disk-center-v1".into(),
        proposal_training_id: "analytic-fixture-proposal-training-v1".into(),
        evaluation_sample_id: evaluation_id.clone(),
        sampling_design_id: "seeded-splitmix-box-muller-synthetic-regression-v1".into(),
        sampling_evidence_ref: "fixture://prng-regression-only/not-production-rng-certification"
            .into(),
        sample_order_digest: sample_order_digest.clone(),
        event_oracle_id: "exact-centered-standard-normal-high-fidelity-disk".into(),
        event_oracle_revision: "closed-form-high-disk-v1".into(),
        event_oracle_is_high_fidelity_continuous_time: false,
        event_oracle_is_analytic_reference_fixture: true,
        proposal_frozen_before_evaluation: true,
        evaluation_draws_are_iid_from_full_mixture: true,
    };
    let surrogate = MultifidelitySurrogateEvidence {
        surrogate_model_id: "exact-oversized-standard-normal-disk-surrogate".into(),
        surrogate_model_revision: "closed-form-low-disk-v1".into(),
        surrogate_training_id: "analytic-surrogate-training-independent-v1".into(),
        surrogate_evaluation_sample_id: evaluation_id,
        target_distribution_id: "whitened-standard-normal-N0I-v1".into(),
        surrogate_probability_evidence_ref: "fixture://surrogate-probability/exact-disk-v1".into(),
        surrogate_probability_artifact_digest: "sha256:analytic-surrogate-probability".into(),
        sample_order_digest,
        surrogate_probability_is_exact: true,
    };
    let surrogate_probability = exact_standard_normal_2d_disk_probability(surrogate_radius)?;
    let report = estimate_multifidelity_residual_probability(
        proposal,
        &whitening(),
        &observations,
        surrogate_probability,
        0.0,
        DELTA,
        &evaluation,
        &surrogate,
    )?;

    Ok(MultifidelityReplicate {
        estimate: report.corrected_high_fidelity_probability_estimate,
        lower: report.confidence_lower,
        upper: report.confidence_upper,
        residual_variance: report.residual_sample_variance,
        direct_variance: report.direct_high_fidelity_sample_variance,
        variance_reduction_ratio: report.direct_to_residual_variance_ratio,
        false_positive_count: report.false_positive_count,
        false_negative_count: report.false_negative_count,
    })
}

fn rms_error(estimates: &[f64], truth: f64) -> f64 {
    (estimates
        .iter()
        .map(|estimate| (estimate - truth).powi(2))
        .sum::<f64>()
        / estimates.len() as f64)
        .sqrt()
}

fn mean(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}

fn main() {
    println!(
        "method,radius,exact_probability,mean_estimate,rmse,mean_lower_95pct,mean_upper_95pct,mean_interval_width,empirical_coverage,mean_variance_ratio,mean_false_positive_count,mean_false_negative_count"
    );
    for (case_index, radius) in [0.10_f64, 0.05, 0.02].into_iter().enumerate() {
        let truth = match exact_standard_normal_2d_disk_probability(radius) {
            Ok(value) => value,
            Err(error) => {
                eprintln!("invalid analytical fixture: {error}");
                process::exit(2);
            }
        };

        // q_adapt = p, so the defensive mixture is exactly the nominal target:
        // this is crude Monte Carlo represented through the same estimator.
        let crude = proposal(
            &format!("crude-target-{case_index}"),
            DEFENSIVE_EPSILON,
            1.0,
        );
        let importance = proposal(
            &format!("defensive-is-{case_index}"),
            DEFENSIVE_EPSILON,
            radius,
        );

        let mut crude_estimates = Vec::with_capacity(REPLICATES);
        let mut crude_upper = Vec::with_capacity(REPLICATES);
        let mut is_estimates = Vec::with_capacity(REPLICATES);
        let mut is_upper = Vec::with_capacity(REPLICATES);
        let mut mf_estimates = Vec::with_capacity(REPLICATES);
        let mut mf_lower = Vec::with_capacity(REPLICATES);
        let mut mf_upper = Vec::with_capacity(REPLICATES);
        let mut mf_variance_ratio = Vec::with_capacity(REPLICATES);
        let mut mf_false_positives = Vec::with_capacity(REPLICATES);
        let mut mf_false_negatives = Vec::with_capacity(REPLICATES);

        for replicate in 0..REPLICATES {
            let crude_seed =
                0x4352_5544_4500_0000_u64 ^ ((case_index as u64) << 32) ^ replicate as u64;
            let is_seed =
                0x4953_4445_4600_0000_u64 ^ ((case_index as u64) << 32) ^ replicate as u64;

            match run_replicate(&crude, radius, crude_seed, replicate, "crude") {
                Ok((estimate, upper)) => {
                    crude_estimates.push(estimate);
                    crude_upper.push(upper);
                }
                Err(error) => {
                    eprintln!("crude benchmark failed: {error}");
                    process::exit(2);
                }
            }
            match run_replicate(&importance, radius, is_seed, replicate, "importance") {
                Ok((estimate, upper)) => {
                    is_estimates.push(estimate);
                    is_upper.push(upper);
                }
                Err(error) => {
                    eprintln!("importance-sampling benchmark failed: {error}");
                    process::exit(2);
                }
            }

            match run_multifidelity_replicate(&importance, radius, is_seed, replicate) {
                Ok(report) => {
                    mf_estimates.push(report.estimate);
                    mf_lower.push(report.lower);
                    mf_upper.push(report.upper);
                    mf_variance_ratio.push(report.variance_reduction_ratio.unwrap_or(0.0));
                    mf_false_positives.push(report.false_positive_count as f64);
                    mf_false_negatives.push(report.false_negative_count as f64);
                }
                Err(error) => {
                    eprintln!("multifidelity benchmark failed: {error}");
                    process::exit(2);
                }
            }
        }

        let crude_rmse = rms_error(&crude_estimates, truth);
        let is_rmse = rms_error(&is_estimates, truth);
        let improvement = if is_rmse > 0.0 {
            crude_rmse / is_rmse
        } else {
            f64::INFINITY
        };

        println!(
            "crude_mc,{radius:.6},{truth:.12},{:.12},{crude_rmse:.12},{:.12},{:.12},{:.12},{:.6},0.0,0.0,0.0",
            mean(&crude_estimates),
            0.0,
            mean(&crude_upper),
            mean(&crude_upper),
        );
        println!(
            "defensive_is,{radius:.6},{truth:.12},{:.12},{is_rmse:.12},{:.12},{:.12},{:.12},{:.6},0.0,0.0,0.0",
            mean(&is_estimates),
            0.0,
            mean(&is_upper),
            mean(&is_upper),
        );

        let mf_rmse = rms_error(&mf_estimates, truth);
        let mf_mean_lower = mean(&mf_lower);
        let mf_mean_upper = mean(&mf_upper);
        let mf_mean_width = mean(
            &mf_lower
                .iter()
                .zip(mf_upper.iter())
                .map(|(lower, upper)| upper - lower)
                .collect::<Vec<_>>(),
        );
        let mf_coverage = mf_lower
            .iter()
            .zip(mf_upper.iter())
            .filter(|(lower, upper)| **lower <= truth && truth <= **upper)
            .count() as f64
            / REPLICATES as f64;
        println!(
            "multifidelity_residual,{radius:.6},{truth:.12},{:.12},{mf_rmse:.12},{mf_mean_lower:.12},{mf_mean_upper:.12},{mf_mean_width:.12},{mf_coverage:.6},{:.6},{:.3},{:.3}",
            mean(&mf_estimates),
            mean(&mf_variance_ratio),
            mean(&mf_false_positives),
            mean(&mf_false_negatives),
        );
        let mf_improvement = if mf_rmse > 0.0 {
            is_rmse / mf_rmse
        } else {
            f64::INFINITY
        };
        eprintln!(
            "radius={radius:.4}: defensive-vs-crude RMSE improvement={improvement:.3}x; residual-vs-direct-IS RMSE improvement={mf_improvement:.3}x; samples/replicate={SAMPLES_PER_REPLICATE}; replicates={REPLICATES}"
        );

        // This assertion is a frozen synthetic regression target, not a claim
        // about a real orbital scenario or a production RNG.
        if !(is_rmse < crude_rmse) {
            eprintln!("defensive IS did not improve RMSE for radius {radius}");
            process::exit(1);
        }
    }
}
