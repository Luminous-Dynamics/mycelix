use gcpn_spatiotemporal_forecast_verification::{
    VariogramPair, crps_ensemble, energy_score, threshold_brier, variogram_score,
};

fn main() {
    let observation = [0.0, 0.0];
    let marginal = [0.0, 1.0];
    let correct_dependence = vec![vec![0.0, 0.0], vec![1.0, 1.0]];
    let wrong_dependence = vec![vec![0.0, 1.0], vec![1.0, 0.0]];
    let pair = [VariogramPair {
        left_index: 0,
        right_index: 1,
        weight: 1.0,
    }];

    println!("GCPN spatiotemporal forecast verification primitives v0.1");
    println!("status: synthetic_joint_field_oracle");
    println!(
        "marginal_crps_correct: {:.8}",
        crps_ensemble(&marginal, 0.0).unwrap()
    );
    println!(
        "marginal_crps_wrong_dependence: {:.8}",
        crps_ensemble(&marginal, 0.0).unwrap()
    );
    println!(
        "threshold_brier: {:.8}",
        threshold_brier(&marginal, 1.0, 0.5).unwrap().0
    );
    println!(
        "energy_score_correct_dependence: {:.8}",
        energy_score(&correct_dependence, &observation).unwrap()
    );
    println!(
        "energy_score_wrong_dependence: {:.8}",
        energy_score(&wrong_dependence, &observation).unwrap()
    );
    println!(
        "variogram_score_correct_dependence: {:.8}",
        variogram_score(&correct_dependence, &observation, 1.0, &pair).unwrap()
    );
    println!(
        "variogram_score_wrong_dependence: {:.8}",
        variogram_score(&wrong_dependence, &observation, 1.0, &pair).unwrap()
    );
    println!(
        "non_claim: synthetic score discrimination is not real forecast skill or SOTA superiority"
    );
}
