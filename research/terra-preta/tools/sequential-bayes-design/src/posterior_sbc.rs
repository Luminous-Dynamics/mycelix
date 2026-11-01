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
