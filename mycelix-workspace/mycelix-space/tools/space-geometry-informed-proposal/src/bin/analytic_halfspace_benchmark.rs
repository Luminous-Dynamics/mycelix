use space_geometry_informed_proposal::{
    DefensiveGeometryProposal, EvaluationEvidence, GeometryMode, GeometryObservation,
    estimate_geometry_event_probability,
};
use std::process;

const SAMPLES: usize = 8_192;
const REPLICATES: usize = 64;
const EPSILON: f64 = 0.05;
const DELTA: f64 = 0.05;
const SQRT_2PI: f64 = 2.506_628_274_631_000_2;

struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    fn new(seed: u64) -> Self { Self { state: seed } }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn uniform_open(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / 9_007_199_254_740_992.0
    }

    fn normal(&mut self) -> f64 {
        let u1 = self.uniform_open();
        let u2 = self.uniform_open();
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }
}

fn phi(x: f64) -> f64 {
    (-0.5 * x * x).exp() / SQRT_2PI
}

// Midpoint quadrature on [a,10]; omitted standard-normal tail beyond 10 is negligible.
fn exact_normal_tail(a: f64) -> f64 {
    let h = 0.0005;
    let count = ((10.0 - a) / h).ceil() as usize;
    (0..count)
        .map(|i| {
            let x = a + (i as f64 + 0.5) * h;
            h * phi(x)
        })
        .sum()
}

fn proposal(threshold: f64, id: &str) -> DefensiveGeometryProposal {
    DefensiveGeometryProposal {
        id: id.to_string(),
        dimension: 1,
        epsilon: EPSILON,
        modes: vec![GeometryMode {
            id: "positive-halfspace-boundary".into(),
            mixture_weight: 1.0,
            normal: vec![1.0],
            tangent_basis: vec![],
            threshold,
            exponential_rate: threshold.max(0.5),
        }],
    }
}

fn crude_mc(seed: u64, threshold: f64) -> f64 {
    let mut rng = SplitMix64::new(seed);
    let hits = (0..SAMPLES).filter(|_| rng.normal() >= threshold).count();
    hits as f64 / SAMPLES as f64
}

fn geometry_is(
    seed: u64,
    threshold: f64,
    proposal: &DefensiveGeometryProposal,
    replicate: usize,
) -> Result<(f64, f64, f64), Box<dyn std::error::Error>> {
    let mut rng = SplitMix64::new(seed);
    let mut observations = Vec::with_capacity(SAMPLES);
    for index in 0..SAMPLES {
        let normal = rng.normal();
        let sample = proposal.sample_from_latents(
            rng.uniform_open(),
            rng.uniform_open(),
            &[normal],
            &[],
            rng.uniform_open(),
        )?;
        let event = sample.point[0] >= threshold;
        observations.push(GeometryObservation {
            sample_id: format!("halfspace-row-{replicate:04}-{index:08}"),
            point: sample.point.clone(),
            generating_component_id: sample.generating_component_id,
            event,
        });
    }
    let evidence = EvaluationEvidence {
        proposal_training_id: "analytic-halfspace-training-v1".into(),
        evaluation_sample_id: format!("analytic-halfspace-evaluation-{seed:016x}"),
        sampling_design_id: "splitmix-box-muller-seeded-synthetic-v1".into(),
        sampling_evidence_ref: "fixture://synthetic-prng-not-production-rng-certification".into(),
        sample_order_digest: format!("fixture-order-{seed:016x}"),
        proposal_frozen_before_evaluation: true,
        evaluation_draws_are_iid_from_full_mixture: true,
    };
    let report = estimate_geometry_event_probability(proposal, &observations, DELTA, &evidence)?;
    Ok((report.estimate, report.upper_confidence_bound, report.event_contribution_ess))
}

fn rmse(values: &[f64], truth: f64) -> f64 {
    (values.iter().map(|value| (value - truth).powi(2)).sum::<f64>() / values.len() as f64).sqrt()
}

fn mean(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}

fn main() {
    println!("threshold,exact_normal_tail,method,mean_estimate,rmse,rmse_gain_vs_crude,mean_upper_95pct,upper_coverage,mean_event_ess,samples_per_replicate,replicates");
    for (case, threshold) in [2.5_f64, 3.5, 4.5].into_iter().enumerate() {
        let truth = exact_normal_tail(threshold);
        let proposal = proposal(threshold, &format!("geo-halfspace-{case}"));
        if let Err(error) = proposal.validate() {
            eprintln!("invalid proposal: {error}");
            process::exit(2);
        }

        let mut crude = Vec::with_capacity(REPLICATES);
        let mut geometric = Vec::with_capacity(REPLICATES);
        let mut uppers = Vec::with_capacity(REPLICATES);
        let mut event_ess = Vec::with_capacity(REPLICATES);

        for replicate in 0..REPLICATES {
            let crude_seed = 0x4352_5544_4500_0000_u64 ^ ((case as u64) << 32) ^ replicate as u64;
            let geo_seed = 0x4745_4f49_5300_0000_u64 ^ ((case as u64) << 32) ^ replicate as u64;
            crude.push(crude_mc(crude_seed, threshold));
            match geometry_is(geo_seed, threshold, &proposal, replicate) {
                Ok((estimate, upper, ess)) => {
                    geometric.push(estimate);
                    uppers.push(upper);
                    event_ess.push(ess);
                }
                Err(error) => {
                    eprintln!("geometry estimator failed: {error}");
                    process::exit(2);
                }
            }
        }

        let crude_rmse = rmse(&crude, truth);
        let geo_rmse = rmse(&geometric, truth);
        let gain = if geo_rmse > 0.0 { crude_rmse / geo_rmse } else { f64::INFINITY };
        let coverage = uppers.iter().filter(|upper| **upper >= truth).count() as f64 / REPLICATES as f64;

        println!(
            "{threshold:.3},{truth:.12},crude_mc,{:.12},{crude_rmse:.12},1.0,0.0,0.0,0.0,{SAMPLES},{REPLICATES}",
            mean(&crude)
        );
        println!(
            "{threshold:.3},{truth:.12},defensive_geo_is,{:.12},{geo_rmse:.12},{gain:.6},{:.12},{coverage:.6},{:.3},{SAMPLES},{REPLICATES}",
            mean(&geometric),
            mean(&uppers),
            mean(&event_ess)
        );
        eprintln!(
            "threshold={threshold:.2}: geometry/defensive RMSE gain={gain:.2}x, exact_tail={truth:.6e}, samples={SAMPLES}, replicates={REPLICATES}"
        );
    }
}
