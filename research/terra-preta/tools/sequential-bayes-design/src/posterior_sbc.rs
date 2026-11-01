//! Finite conditional posterior-SBC reference controls.
//!
//! These tests implement Posterior SBC for a finite categorical model and a
//! fixed observed-data fixture. They are calibration checks for exact discrete
//! Bayes updating, not a continuous posterior sampler or physical-model
//! adequacy test.

#[cfg(test)]
mod tests {
    const REPLICATES: usize = 4_096;
    const BIN_COUNT: usize = 10;
    const LOWER_FREQUENCY: f64 = 0.07;
    const UPPER_FREQUENCY: f64 = 0.13;

    struct SplitMix64 {
        state: u64,
    }

    impl SplitMix64 {
        fn new(seed: u64) -> Self {
            Self { state: seed }
        }

        fn next_u64(&mut self) -> u64 {
            self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut value = self.state;
            value = (value ^ (value >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            value = (value ^ (value >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            value ^ (value >> 31)
        }

        fn uniform_open01(&mut self) -> f64 {
            ((self.next_u64() >> 11) as f64 + 0.5) / 9_007_199_254_740_992.0
        }
    }

    fn sample_category(probabilities: &[f64], rng: &mut SplitMix64) -> usize {
        let draw = rng.uniform_open01();
        let mut cumulative = 0.0;
        for (index, probability) in probabilities.iter().enumerate() {
            cumulative += probability;
            if draw < cumulative {
                return index;
            }
        }
        probabilities.len() - 1
    }

    /// Randomized discrete PIT of a selected state under the supplied posterior.
    fn randomized_pit(
        values: &[f64],
        posterior: &[f64],
        selected_index: usize,
        rng: &mut SplitMix64,
    ) -> f64 {
        let selected_value = values[selected_index];
        let mut below = 0.0;
        let mut tied = 0.0;
        for (value, weight) in values.iter().zip(posterior) {
            if *value < selected_value {
                below += weight;
            } else if *value == selected_value {
                tied += weight;
            }
        }
        below + rng.uniform_open01() * tied
    }

    fn add_to_histogram(histogram: &mut [usize; BIN_COUNT], pit: f64) {
        assert!(pit.is_finite() && (0.0..=1.0).contains(&pit));
        let bin = ((pit * BIN_COUNT as f64).floor() as usize).min(BIN_COUNT - 1);
        histogram[bin] += 1;
    }

    fn posterior_sbc_histograms(
        update_likelihood_power: i32,
    ) -> ([usize; BIN_COUNT], [usize; BIN_COUNT]) {
        // Fixed observed-data fixture: prior multiplied by the likelihood of
        // y_obs, producing a new conditional prior for posterior SBC.
        let prior: [f64; 3] = [0.2, 0.5, 0.3];
        let observed_data_likelihood: [f64; 3] = [0.2, 0.8, 0.4];
        let unnormalized: Vec<f64> = prior.iter()
            .zip(observed_data_likelihood)
            .map(|(weight, likelihood)| weight * likelihood)
            .collect();
        let normalizer: f64 = unnormalized.iter().sum();
        let conditional_prior: Vec<f64> =
            unnormalized.iter().map(|weight| weight / normalizer).collect();

        // Two possible posterior-predictive outcomes for each latent model.
        let likelihoods: [[f64; 2]; 3] = [[0.9, 0.1], [0.5, 0.5], [0.1, 0.9]];
        let joint_log_likelihood = |model: usize, outcome: usize| {
            observed_data_likelihood[model].ln() + likelihoods[model][outcome].ln()
        };

        let mut parameter_ranks = [0usize; BIN_COUNT];
        let mut joint_likelihood_ranks = [0usize; BIN_COUNT];
        let mut rng = SplitMix64::new(2026_1101);

        for _ in 0..REPLICATES {
            // Original draw from p(M | y_obs), followed by a posterior
            // predictive observation y_new ~ p(y_new | M, y_obs).
            let original_model = sample_category(&conditional_prior, &mut rng);
            let new_observation = sample_category(&likelihoods[original_model], &mut rng);

            // The augmented posterior p(M | y_obs, y_new). Power=1 is exact;
            // power=4 is an intentionally overconfident negative control.
            let augmented_unnormalized: Vec<f64> = (0..prior.len())
                .map(|model| {
                    conditional_prior[model]
                        * likelihoods[model][new_observation].powi(update_likelihood_power)
                })
                .collect();
            let augmented_normalizer: f64 = augmented_unnormalized.iter().sum();
            let augmented_posterior: Vec<f64> = augmented_unnormalized.iter()
                .map(|weight| weight / augmented_normalizer)
                .collect();

            // Test quantity 1: model identity (discrete randomized PIT).
            let model_values = [0.0, 1.0, 2.0];
            let model_pit = randomized_pit(
                &model_values,
                &augmented_posterior,
                original_model,
                &mut rng,
            );
            add_to_histogram(&mut parameter_ranks, model_pit);

            // Test quantity 2: joint log likelihood of the fixed and new data.
            // This detects inference that ignores or misweights observations.
            let log_likelihood_values: Vec<f64> = (0..prior.len())
                .map(|model| joint_log_likelihood(model, new_observation))
                .collect();
            let likelihood_pit = randomized_pit(
                &log_likelihood_values,
                &augmented_posterior,
                original_model,
                &mut rng,
            );
            add_to_histogram(&mut joint_likelihood_ranks, likelihood_pit);
        }

        (parameter_ranks, joint_likelihood_ranks)
    }

    fn assert_uniform_histogram(histogram: &[usize; BIN_COUNT]) {
        let total = histogram.iter().sum::<usize>() as f64;
        assert_eq!(total as usize, REPLICATES);
        for (bin, count) in histogram.iter().enumerate() {
            let frequency = *count as f64 / total;
            assert!(
                (LOWER_FREQUENCY..=UPPER_FREQUENCY).contains(&frequency),
                "posterior-SBC bin {bin} has frequency {frequency:.4}; counts={histogram:?}"
            );
        }
    }


    fn standard_normal(rng: &mut SplitMix64) -> f64 {
        // Box-Muller transform using open-interval uniforms.
        let radius = (-2.0 * rng.uniform_open01().ln()).sqrt();
        let angle = std::f64::consts::TAU * rng.uniform_open01();
        radius * angle.cos()
    }

    fn normal_cdf(value: f64) -> f64 {
        // Abramowitz-Stegun 7.1.26 approximation; absolute error is below
        // 8e-8 and is negligible relative to the 4096-draw rank histogram.
        let sign = if value < 0.0 { -1.0 } else { 1.0 };
        let x = value.abs();
        let t = 1.0 / (1.0 + 0.231_641_9 * x);
        let density = 0.398_942_280_401_432_7 * (-0.5 * x * x).exp();
        let polynomial = t
            * (0.319_381_530
                + t * (-0.356_563_782
                    + t * (1.781_477_937
                        + t * (-1.821_255_978 + t * 1.330_274_429))));
        let upper_tail = density * polynomial;
        if sign > 0.0 { 1.0 - upper_tail } else { upper_tail }
    }

    // Conjugate Normal-Normal posterior for a known observation variance.
    // A likelihood power other than one is an intentionally misspecified update.
    fn gaussian_posterior(
        prior_mean: f64,
        prior_variance: f64,
        observation_variance: f64,
        observations: &[f64],
        likelihood_power: f64,
    ) -> (f64, f64) {
        let prior_precision = 1.0 / prior_variance;
        let observation_precision =
            observations.len() as f64 * likelihood_power / observation_variance;
        let posterior_variance = 1.0 / (prior_precision + observation_precision);
        let weighted_observations = observations.iter().sum::<f64>()
            * likelihood_power / observation_variance;
        let posterior_mean =
            (prior_mean * prior_precision + weighted_observations)
                * posterior_variance;
        (posterior_mean, posterior_variance)
    }

    fn continuous_posterior_sbc_histogram(
        augmented_likelihood_power: f64,
    ) -> [usize; BIN_COUNT] {
        // One fixed observed dataset conditions the first posterior. This is
        // a conjugate reference, not an MCMC or differential-equation sampler.
        let prior_mean = -0.3;
        let prior_variance = 1.2;
        let observation_variance = 0.2;
        let observed_y = 1.4;
        let (conditional_mean, conditional_variance) = gaussian_posterior(
            prior_mean,
            prior_variance,
            observation_variance,
            &[observed_y],
            1.0,
        );

        let mut histogram = [0usize; BIN_COUNT];
        let mut rng = SplitMix64::new(2026_1102);
        for _ in 0..REPLICATES {
            // theta' ~ p(theta | y_obs), then y_new ~ p(y_new | theta').
            let original_theta = conditional_mean
                + conditional_variance.sqrt() * standard_normal(&mut rng);
            let new_observation = original_theta
                + observation_variance.sqrt() * standard_normal(&mut rng);

            // Reference inference uses both observed values. Raising the
            // likelihood power deliberately creates an overconfident control.
            let (augmented_mean, augmented_variance) = gaussian_posterior(
                prior_mean,
                prior_variance,
                observation_variance,
                &[observed_y, new_observation],
                augmented_likelihood_power,
            );
            let pit = normal_cdf(
                (original_theta - augmented_mean) / augmented_variance.sqrt(),
            );
            add_to_histogram(&mut histogram, pit);
        }
        histogram
    }

    #[test]
    fn conjugate_gaussian_posterior_sbc_passes_for_fixed_observed_data() {
        let histogram = continuous_posterior_sbc_histogram(1.0);
        assert_uniform_histogram(&histogram);
    }

    #[test]
    fn conjugate_gaussian_posterior_sbc_detects_overconfident_likelihood_power() {
        let histogram = continuous_posterior_sbc_histogram(4.0);
        assert!(
            histogram.iter().any(|count| {
                let frequency = *count as f64 / REPLICATES as f64;
                !(LOWER_FREQUENCY..=UPPER_FREQUENCY).contains(&frequency)
            }),
            "overconfident Gaussian update should fail posterior SBC: {histogram:?}"
        );
    }

    #[test]
    fn exact_posterior_update_passes_conditional_posterior_sbc() {
        let (model_ranks, joint_likelihood_ranks) = posterior_sbc_histograms(1);
        assert_uniform_histogram(&model_ranks);
        assert_uniform_histogram(&joint_likelihood_ranks);
    }

    #[test]
    fn overconfident_augmented_update_fails_conditional_posterior_sbc() {
        let (model_ranks, joint_likelihood_ranks) = posterior_sbc_histograms(4);
        assert!(
            model_ranks.iter().any(|count| {
                let frequency = *count as f64 / REPLICATES as f64;
                !(LOWER_FREQUENCY..=UPPER_FREQUENCY).contains(&frequency)
            }) || joint_likelihood_ranks.iter().any(|count| {
                let frequency = *count as f64 / REPLICATES as f64;
                !(LOWER_FREQUENCY..=UPPER_FREQUENCY).contains(&frequency)
            }),
            "overconfident update should fail at least one posterior-SBC histogram: model={model_ranks:?}, likelihood={joint_likelihood_ranks:?}"
        );
    }
}
