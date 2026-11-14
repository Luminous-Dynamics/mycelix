#![forbid(unsafe_code)]

use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Hypothesis {
    pub id: String,
    pub prior_weight: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Action {
    pub id: String,
    /// Loss values are ordered exactly like DecisionCase::hypotheses.
    pub loss_by_hypothesis: Vec<u64>,
    /// False means the action is excluded by the hard constraint layer.
    pub admissible: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Outcome {
    pub id: String,
    /// P(outcome | hypothesis, measurement), represented as integer weights.
    /// For each hypothesis, weights across all outcomes must sum to likelihood_scale.
    pub likelihood_weight_by_hypothesis: Vec<u64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Measurement {
    pub id: String,
    pub authorised: bool,
    pub safe: bool,
    /// Acquisition and delay costs use the same loss units as Action::loss_by_hypothesis.
    pub acquisition_cost: u64,
    pub delay_cost: u64,
    pub outcomes: Vec<Outcome>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecisionCase {
    pub likelihood_scale: u64,
    pub hypotheses: Vec<Hypothesis>,
    pub actions: Vec<Action>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rational {
    pub numerator: u128,
    pub denominator: u128,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SignedRational {
    pub numerator: i128,
    pub denominator: u128,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeasurementDisposition {
    EvaluatedPositiveNetValue,
    EvaluatedNonPositiveNetValue,
    BlockedUnauthorised,
    BlockedUnsafe,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OutcomeChoice {
    pub outcome_id: String,
    pub action_id: String,
    pub predictive_mass: u128,
    pub posterior_expected_loss: Rational,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MeasurementResult {
    pub measurement_id: String,
    pub disposition: MeasurementDisposition,
    pub baseline_action_id: Option<String>,
    pub baseline_expected_loss: Option<Rational>,
    pub outcome_choices: Vec<OutcomeChoice>,
    pub expected_post_measurement_loss: Option<Rational>,
    pub gross_evsi: Option<Rational>,
    pub acquisition_cost: u64,
    pub delay_cost: u64,
    pub net_evsi: Option<SignedRational>,
    /// True only for the single highest strictly positive net-EVSI candidate
    /// selected for the next acquisition from this frozen evidence snapshot.
    pub selected: bool,
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn checked_add(left: u128, right: u128, label: &str) -> Result<u128, String> {
    left.checked_add(right)
        .ok_or_else(|| format!("arithmetic overflow while adding {label}"))
}

fn checked_mul(left: u128, right: u128, label: &str) -> Result<u128, String> {
    left.checked_mul(right)
        .ok_or_else(|| format!("arithmetic overflow while multiplying {label}"))
}

fn validate_case(case: &DecisionCase) -> Result<u128, String> {
    if case.likelihood_scale == 0 {
        return Err("likelihood_scale must be positive".to_owned());
    }
    if case.hypotheses.is_empty() {
        return Err("at least one hypothesis is required".to_owned());
    }
    if case.actions.is_empty() {
        return Err("at least one action is required".to_owned());
    }

    let mut hypothesis_ids = BTreeSet::new();
    let mut prior_total = 0_u128;
    for hypothesis in &case.hypotheses {
        if !nonempty(&hypothesis.id) {
            return Err("hypothesis IDs must be non-empty".to_owned());
        }
        if !hypothesis_ids.insert(hypothesis.id.as_str()) {
            return Err(format!("duplicate hypothesis ID {}", hypothesis.id));
        }
        prior_total = checked_add(
            prior_total,
            u128::from(hypothesis.prior_weight),
            "prior weights",
        )?;
    }

    if prior_total == 0 {
        return Err("at least one hypothesis must have positive prior weight".to_owned());
    }

    let mut action_ids = BTreeSet::new();
    let mut has_no_action = false;
    let mut has_admissible_action = false;
    for action in &case.actions {
        if !nonempty(&action.id) {
            return Err("action IDs must be non-empty".to_owned());
        }
        if !action_ids.insert(action.id.as_str()) {
            return Err(format!("duplicate action ID {}", action.id));
        }
        if action.loss_by_hypothesis.len() != case.hypotheses.len() {
            return Err(format!(
                "action {} has {} losses for {} hypotheses",
                action.id,
                action.loss_by_hypothesis.len(),
                case.hypotheses.len()
            ));
        }
        if action.id == "NO_ACTION" && action.admissible {
            has_no_action = true;
        }
        has_admissible_action |= action.admissible;
    }
    if !has_no_action {
        return Err("an admissible NO_ACTION option is required".to_owned());
    }
    if !has_admissible_action {
        return Err("at least one action must be admissible".to_owned());
    }
    Ok(prior_total)
}

fn validate_measurements(case: &DecisionCase, measurements: &[Measurement]) -> Result<(), String> {
    if measurements.is_empty() {
        return Err("at least one measurement is required".to_owned());
    }
    let mut measurement_ids = BTreeSet::new();
    for measurement in measurements {
        if !nonempty(&measurement.id) {
            return Err("measurement IDs must be non-empty".to_owned());
        }
        if !measurement_ids.insert(measurement.id.as_str()) {
            return Err(format!("duplicate measurement ID {}", measurement.id));
        }
        if measurement.outcomes.is_empty() {
            return Err(format!("measurement {} has no outcomes", measurement.id));
        }

        let mut outcome_ids = BTreeSet::new();
        let mut likelihood_totals = vec![0_u128; case.hypotheses.len()];
        for outcome in &measurement.outcomes {
            if !nonempty(&outcome.id) {
                return Err(format!(
                    "measurement {} has an empty outcome ID",
                    measurement.id
                ));
            }
            if !outcome_ids.insert(outcome.id.as_str()) {
                return Err(format!(
                    "measurement {} has duplicate outcome ID {}",
                    measurement.id, outcome.id
                ));
            }
            if outcome.likelihood_weight_by_hypothesis.len() != case.hypotheses.len() {
                return Err(format!(
                    "outcome {} of measurement {} has a likelihood vector with the wrong length",
                    outcome.id, measurement.id
                ));
            }
            for (index, likelihood) in outcome.likelihood_weight_by_hypothesis.iter().enumerate() {
                likelihood_totals[index] = checked_add(
                    likelihood_totals[index],
                    u128::from(*likelihood),
                    "likelihood totals",
                )?;
            }
        }

        for (index, total) in likelihood_totals.iter().enumerate() {
            if *total != u128::from(case.likelihood_scale) {
                return Err(format!(
                    "measurement {} likelihoods for hypothesis {} sum to {}, expected {}",
                    measurement.id, case.hypotheses[index].id, total, case.likelihood_scale
                ));
            }
        }
    }
    Ok(())
}

fn minimum_loss_action(
    case: &DecisionCase,
    joint_weights: &[u128],
) -> Result<(String, u128), String> {
    let mut best: Option<(String, u128)> = None;
    for action in case.actions.iter().filter(|action| action.admissible) {
        let mut weighted_loss = 0_u128;
        for (index, loss) in action.loss_by_hypothesis.iter().enumerate() {
            let term = checked_mul(
                joint_weights[index],
                u128::from(*loss),
                "weighted action loss",
            )?;
            weighted_loss = checked_add(weighted_loss, term, "weighted action loss")?;
        }
        let replace = match &best {
            None => true,
            Some((best_id, best_loss)) => {
                weighted_loss < *best_loss || (weighted_loss == *best_loss && action.id < *best_id)
            }
        };
        if replace {
            best = Some((action.id.clone(), weighted_loss));
        }
    }
    best.ok_or_else(|| "no admissible action remains".to_owned())
}

fn blocked_result(
    measurement: &Measurement,
    disposition: MeasurementDisposition,
) -> MeasurementResult {
    MeasurementResult {
        measurement_id: measurement.id.clone(),
        disposition,
        baseline_action_id: None,
        baseline_expected_loss: None,
        outcome_choices: Vec::new(),
        expected_post_measurement_loss: None,
        gross_evsi: None,
        acquisition_cost: measurement.acquisition_cost,
        delay_cost: measurement.delay_cost,
        net_evsi: None,
        selected: false,
    }
}

/// Evaluate measurements using exact integer probability weights and loss units.
///
/// The action set is fixed by the pre-evaluation hard constraint layer. This v0.1
/// reference does not allow an observation to create new authority or make an
/// otherwise inadmissible action eligible.
pub fn evaluate(
    case: &DecisionCase,
    measurements: &[Measurement],
) -> Result<Vec<MeasurementResult>, String> {
    let prior_total = validate_case(case)?;
    validate_measurements(case, measurements)?;

    let baseline_joint_weights: Vec<u128> = case
        .hypotheses
        .iter()
        .map(|hypothesis| u128::from(hypothesis.prior_weight))
        .collect();
    let (baseline_action_id, baseline_best_loss) =
        minimum_loss_action(case, &baseline_joint_weights)?;
    let combined_denominator = checked_mul(
        prior_total,
        u128::from(case.likelihood_scale),
        "combined expected-loss denominator",
    )?;

    let mut results = Vec::with_capacity(measurements.len());
    for measurement in measurements {
        if !measurement.authorised {
            results.push(blocked_result(
                measurement,
                MeasurementDisposition::BlockedUnauthorised,
            ));
            continue;
        }
        if !measurement.safe {
            results.push(blocked_result(
                measurement,
                MeasurementDisposition::BlockedUnsafe,
            ));
            continue;
        }

        let mut post_measurement_loss_numerator = 0_u128;
        let mut outcome_choices = Vec::with_capacity(measurement.outcomes.len());
        for outcome in &measurement.outcomes {
            let joint_weights: Vec<u128> = case
                .hypotheses
                .iter()
                .zip(&outcome.likelihood_weight_by_hypothesis)
                .map(|(hypothesis, likelihood)| {
                    checked_mul(
                        u128::from(hypothesis.prior_weight),
                        u128::from(*likelihood),
                        "joint hypothesis/outcome weight",
                    )
                })
                .collect::<Result<_, _>>()?;
            let predictive_mass = joint_weights.iter().try_fold(0_u128, |sum, mass| {
                checked_add(sum, *mass, "predictive mass")
            })?;
            // Outcomes impossible under the current posterior contribute no expected loss.
            // They are omitted from the conditional recommendation, not treated as errors.
            if predictive_mass == 0 {
                continue;
            }
            let (action_id, best_joint_loss) = minimum_loss_action(case, &joint_weights)?;
            post_measurement_loss_numerator = checked_add(
                post_measurement_loss_numerator,
                best_joint_loss,
                "post-measurement expected-loss numerator",
            )?;
            outcome_choices.push(OutcomeChoice {
                outcome_id: outcome.id.clone(),
                action_id,
                predictive_mass,
                posterior_expected_loss: Rational {
                    numerator: best_joint_loss,
                    denominator: predictive_mass,
                },
            });
        }

        let scaled_baseline = checked_mul(
            baseline_best_loss,
            u128::from(case.likelihood_scale),
            "scaled baseline loss",
        )?;
        let gross_numerator = scaled_baseline
            .checked_sub(post_measurement_loss_numerator)
            .ok_or_else(|| {
                "post-measurement expected loss exceeds baseline; check likelihood/action inputs"
                    .to_owned()
            })?;
        let total_cost = measurement
            .acquisition_cost
            .checked_add(measurement.delay_cost)
            .ok_or_else(|| "measurement acquisition plus delay cost overflow".to_owned())?;
        let cost_numerator = checked_mul(
            u128::from(total_cost),
            combined_denominator,
            "weighted measurement cost",
        )?;
        let gross_signed = i128::try_from(gross_numerator)
            .map_err(|_| "gross EVSI exceeds signed rational range".to_owned())?;
        let cost_signed = i128::try_from(cost_numerator)
            .map_err(|_| "measurement cost exceeds signed rational range".to_owned())?;
        let net_numerator = gross_signed
            .checked_sub(cost_signed)
            .ok_or_else(|| "net EVSI signed arithmetic overflow".to_owned())?;

        let disposition = if net_numerator > 0 {
            MeasurementDisposition::EvaluatedPositiveNetValue
        } else {
            MeasurementDisposition::EvaluatedNonPositiveNetValue
        };
        results.push(MeasurementResult {
            measurement_id: measurement.id.clone(),
            disposition,
            baseline_action_id: Some(baseline_action_id.clone()),
            baseline_expected_loss: Some(Rational {
                numerator: baseline_best_loss,
                denominator: prior_total,
            }),
            outcome_choices,
            expected_post_measurement_loss: Some(Rational {
                numerator: post_measurement_loss_numerator,
                denominator: combined_denominator,
            }),
            gross_evsi: Some(Rational {
                numerator: gross_numerator,
                denominator: combined_denominator,
            }),
            acquisition_cost: measurement.acquisition_cost,
            delay_cost: measurement.delay_cost,
            net_evsi: Some(SignedRational {
                numerator: net_numerator,
                denominator: combined_denominator,
            }),
            selected: false,
        });
    }

    results.sort_by(|left, right| left.measurement_id.cmp(&right.measurement_id));

    // All candidates are evaluated against the same frozen evidence snapshot.
    // Their individual net values are not additive. Select only the best strictly
    // positive candidate; sorted IDs provide a deterministic lexical tie-break.
    let mut best_positive_index: Option<(usize, i128)> = None;
    for (index, result) in results.iter().enumerate() {
        let Some(net) = result.net_evsi else {
            continue;
        };
        if net.numerator <= 0 {
            continue;
        }
        if best_positive_index
            .map(|(_, best_numerator)| net.numerator > best_numerator)
            .unwrap_or(true)
        {
            best_positive_index = Some((index, net.numerator));
        }
    }
    if let Some((index, _)) = best_positive_index {
        results[index].selected = true;
    }
    Ok(results)
}

/// Condition a finite-hypothesis case on an observed outcome from one authorised,
/// safe measurement. The caller must bind the outcome ID to an owner-qualified
/// observation receipt; this function performs probability arithmetic only.
pub fn condition_case_on_outcome(
    case: &DecisionCase,
    measurement: &Measurement,
    observed_outcome_id: &str,
) -> Result<DecisionCase, String> {
    validate_case(case)?;
    validate_measurements(case, std::slice::from_ref(measurement))?;
    if !measurement.authorised {
        return Err("cannot condition on an unauthorised measurement".to_owned());
    }
    if !measurement.safe {
        return Err("cannot condition on an unsafe measurement".to_owned());
    }
    let outcome = measurement
        .outcomes
        .iter()
        .find(|outcome| outcome.id == observed_outcome_id)
        .ok_or_else(|| format!("unknown observed outcome {observed_outcome_id}"))?;

    let joint_weights: Vec<u128> = case
        .hypotheses
        .iter()
        .zip(&outcome.likelihood_weight_by_hypothesis)
        .map(|(hypothesis, likelihood)| {
            checked_mul(
                u128::from(hypothesis.prior_weight),
                u128::from(*likelihood),
                "posterior hypothesis weight",
            )
        })
        .collect::<Result<_, _>>()?;
    let predictive_mass = joint_weights.iter().try_fold(0_u128, |sum, weight| {
        checked_add(sum, *weight, "posterior mass")
    })?;
    if predictive_mass == 0 {
        return Err(format!(
            "observed outcome {observed_outcome_id} is impossible under the current hypothesis weights"
        ));
    }

    let divisor = joint_weights
        .iter()
        .copied()
        .filter(|weight| *weight > 0)
        .fold(0_u128, gcd);
    let mut posterior = case.clone();
    for (hypothesis, joint_weight) in posterior.hypotheses.iter_mut().zip(joint_weights) {
        let reduced = joint_weight / divisor;
        hypothesis.prior_weight = u64::try_from(reduced)
            .map_err(|_| "reduced posterior weight exceeds u64".to_owned())?;
    }
    validate_case(&posterior)?;
    Ok(posterior)
}

fn gcd(left: u128, right: u128) -> u128 {
    let mut a = left;
    let mut b = right;
    while b != 0 {
        let remainder = a % b;
        a = b;
        b = remainder;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decision_case() -> DecisionCase {
        DecisionCase {
            likelihood_scale: 100,
            hypotheses: vec![
                Hypothesis {
                    id: "blocked-sewer".to_owned(),
                    prior_weight: 50,
                },
                Hypothesis {
                    id: "pump-failure".to_owned(),
                    prior_weight: 50,
                },
            ],
            actions: vec![
                Action {
                    id: "ACTION_A".to_owned(),
                    loss_by_hypothesis: vec![0, 100],
                    admissible: true,
                },
                Action {
                    id: "ACTION_B".to_owned(),
                    loss_by_hypothesis: vec![100, 0],
                    admissible: true,
                },
                Action {
                    id: "NO_ACTION".to_owned(),
                    loss_by_hypothesis: vec![60, 60],
                    admissible: true,
                },
                Action {
                    id: "UNAUTHORISED_REPAIR".to_owned(),
                    loss_by_hypothesis: vec![0, 0],
                    admissible: false,
                },
            ],
        }
    }

    fn discriminating_measurement(id: &str) -> Measurement {
        Measurement {
            id: id.to_owned(),
            authorised: true,
            safe: true,
            acquisition_cost: 5,
            delay_cost: 2,
            outcomes: vec![
                Outcome {
                    id: "red".to_owned(),
                    likelihood_weight_by_hypothesis: vec![90, 10],
                },
                Outcome {
                    id: "blue".to_owned(),
                    likelihood_weight_by_hypothesis: vec![10, 90],
                },
            ],
        }
    }

    #[test]
    fn exact_evsi_selects_a_measurement_that_changes_the_decision() {
        let report = evaluate(
            &decision_case(),
            &[discriminating_measurement("flow-check")],
        )
        .expect("synthetic case should validate");
        let result = &report[0];
        assert_eq!(
            result.disposition,
            MeasurementDisposition::EvaluatedPositiveNetValue
        );
        assert_eq!(result.baseline_action_id.as_deref(), Some("ACTION_A"));
        assert_eq!(
            result.expected_post_measurement_loss,
            Some(Rational {
                numerator: 100_000,
                denominator: 10_000
            })
        );
        assert_eq!(
            result.gross_evsi,
            Some(Rational {
                numerator: 400_000,
                denominator: 10_000
            })
        );
        assert_eq!(
            result.net_evsi,
            Some(SignedRational {
                numerator: 330_000,
                denominator: 10_000
            })
        );
        assert!(result.selected);
        assert_eq!(result.outcome_choices[0].action_id, "ACTION_A");
        assert_eq!(result.outcome_choices[1].action_id, "ACTION_B");
    }

    #[test]
    fn high_information_measurement_with_no_decision_value_is_not_selected() {
        let case = DecisionCase {
            likelihood_scale: 100,
            hypotheses: vec![
                Hypothesis {
                    id: "cause-a".to_owned(),
                    prior_weight: 50,
                },
                Hypothesis {
                    id: "cause-b".to_owned(),
                    prior_weight: 50,
                },
            ],
            actions: vec![
                Action {
                    id: "NO_ACTION".to_owned(),
                    loss_by_hypothesis: vec![0, 0],
                    admissible: true,
                },
                Action {
                    id: "repair-a".to_owned(),
                    loss_by_hypothesis: vec![100, 120],
                    admissible: true,
                },
                Action {
                    id: "repair-b".to_owned(),
                    loss_by_hypothesis: vec![120, 100],
                    admissible: true,
                },
            ],
        };
        let measurement = Measurement {
            id: "perfect-diagnostic".to_owned(),
            authorised: true,
            safe: true,
            acquisition_cost: 0,
            delay_cost: 0,
            outcomes: vec![
                Outcome {
                    id: "a".to_owned(),
                    likelihood_weight_by_hypothesis: vec![100, 0],
                },
                Outcome {
                    id: "b".to_owned(),
                    likelihood_weight_by_hypothesis: vec![0, 100],
                },
            ],
        };
        let result = &evaluate(&case, &[measurement]).expect("case should validate")[0];
        assert_eq!(
            result.disposition,
            MeasurementDisposition::EvaluatedNonPositiveNetValue
        );
        assert_eq!(
            result.gross_evsi,
            Some(Rational {
                numerator: 0,
                denominator: 10_000
            })
        );
        assert!(!result.selected);
        assert_eq!(result.outcome_choices[0].action_id, "NO_ACTION");
        assert_eq!(result.outcome_choices[1].action_id, "NO_ACTION");
    }

    #[test]
    fn measurement_without_authorisation_is_blocked_without_evsi() {
        let mut measurement = discriminating_measurement("unauthorised-inspection");
        measurement.authorised = false;
        let result = &evaluate(&decision_case(), &[measurement]).expect("case should validate")[0];
        assert_eq!(
            result.disposition,
            MeasurementDisposition::BlockedUnauthorised
        );
        assert_eq!(result.gross_evsi, None);
        assert_eq!(result.net_evsi, None);
        assert!(!result.selected);
    }

    #[test]
    fn unsafe_measurement_is_blocked_without_evsi() {
        let mut measurement = discriminating_measurement("unsafe-sampling");
        measurement.safe = false;
        let result = &evaluate(&decision_case(), &[measurement]).expect("case should validate")[0];
        assert_eq!(result.disposition, MeasurementDisposition::BlockedUnsafe);
        assert_eq!(result.gross_evsi, None);
        assert_eq!(result.net_evsi, None);
        assert!(!result.selected);
    }

    #[test]
    fn likelihood_weights_must_sum_to_declared_scale_for_every_hypothesis() {
        let mut measurement = discriminating_measurement("bad-likelihood");
        measurement.outcomes[0].likelihood_weight_by_hypothesis[0] = 89;
        assert!(evaluate(&decision_case(), &[measurement])
            .unwrap_err()
            .contains("expected 100"));
    }

    #[test]
    fn missing_no_action_option_is_rejected() {
        let mut case = decision_case();
        case.actions.retain(|action| action.id != "NO_ACTION");
        assert!(evaluate(&case, &[discriminating_measurement("test")])
            .unwrap_err()
            .contains("NO_ACTION"));
    }

    #[test]
    fn inadmissible_actions_do_not_influence_the_recommended_decision() {
        let mut case = decision_case();
        case.actions.push(Action {
            id: "FORBIDDEN_ZERO_LOSS".to_owned(),
            loss_by_hypothesis: vec![0, 0],
            admissible: false,
        });
        let result = &evaluate(&case, &[discriminating_measurement("test")])
            .expect("case should validate")[0];
        assert_eq!(result.baseline_action_id.as_deref(), Some("ACTION_A"));
        assert!(!result
            .outcome_choices
            .iter()
            .any(|choice| choice.action_id == "FORBIDDEN_ZERO_LOSS"));
    }

    #[test]
    fn duplicate_hypothesis_and_measurement_ids_are_rejected() {
        let mut case = decision_case();
        case.hypotheses.push(case.hypotheses[0].clone());
        assert!(evaluate(&case, &[discriminating_measurement("test")])
            .unwrap_err()
            .contains("duplicate hypothesis"));

        let case = decision_case();
        assert!(evaluate(
            &case,
            &[
                discriminating_measurement("duplicate"),
                discriminating_measurement("duplicate")
            ]
        )
        .unwrap_err()
        .contains("duplicate measurement"));
    }

    #[test]
    fn zero_predictive_probability_outcomes_are_omitted_from_recommendations() {
        let mut measurement = discriminating_measurement("zero-outcome");
        measurement.outcomes.push(Outcome {
            id: "impossible".to_owned(),
            likelihood_weight_by_hypothesis: vec![0, 0],
        });
        let result = &evaluate(&decision_case(), &[measurement])
            .expect("impossible outcomes should not invalidate a measurement")[0];
        assert_eq!(result.outcome_choices.len(), 2);
        assert!(result
            .outcome_choices
            .iter()
            .all(|choice| choice.outcome_id != "impossible"));
    }

    #[test]
    fn likelihood_and_prior_scale_changes_preserve_net_evsi() {
        let mut case = decision_case();
        let mut measurement = discriminating_measurement("scale-check");
        let original = &evaluate(&case, &[measurement.clone()]).expect("original scale")[0];
        let original_net = original.net_evsi.expect("net EVSI");

        case.likelihood_scale *= 2;
        for hypothesis in &mut case.hypotheses {
            hypothesis.prior_weight *= 2;
        }
        for outcome in &mut measurement.outcomes {
            for likelihood in &mut outcome.likelihood_weight_by_hypothesis {
                *likelihood *= 2;
            }
        }
        let scaled = &evaluate(&case, &[measurement]).expect("scaled weights")[0]
            .net_evsi
            .expect("scaled net EVSI");
        assert_eq!(
            original_net.numerator * i128::try_from(scaled.denominator).unwrap(),
            scaled.numerator * i128::try_from(original_net.denominator).unwrap()
        );
    }

    #[test]
    fn uninformative_measurement_has_zero_decision_value() {
        let measurement = Measurement {
            id: "no-information".to_owned(),
            authorised: true,
            safe: true,
            acquisition_cost: 0,
            delay_cost: 0,
            outcomes: vec![
                Outcome {
                    id: "outcome-a".to_owned(),
                    likelihood_weight_by_hypothesis: vec![60, 60],
                },
                Outcome {
                    id: "outcome-b".to_owned(),
                    likelihood_weight_by_hypothesis: vec![40, 40],
                },
            ],
        };
        let result = &evaluate(&decision_case(), &[measurement])
            .expect("uninformative measurement should validate")[0];
        assert_eq!(
            result.gross_evsi,
            Some(Rational {
                numerator: 0,
                denominator: 10_000
            })
        );
        assert!(!result.selected);
    }

    #[test]
    fn exact_evsi_matches_full_signal_quality_sweep() {
        let case = decision_case();
        let expected_denominator = 10_000_u128;
        for signal_quality in 0_u64..=100 {
            let measurement = Measurement {
                id: format!("signal-quality-{signal_quality:03}"),
                authorised: true,
                safe: true,
                acquisition_cost: 0,
                delay_cost: 0,
                outcomes: vec![
                    Outcome {
                        id: "red".to_owned(),
                        likelihood_weight_by_hypothesis: vec![signal_quality, 100 - signal_quality],
                    },
                    Outcome {
                        id: "blue".to_owned(),
                        likelihood_weight_by_hypothesis: vec![100 - signal_quality, signal_quality],
                    },
                ],
            };
            let result =
                &evaluate(&case, &[measurement]).expect("signal-quality case should validate")[0];
            let expected_numerator = u128::from(signal_quality.abs_diff(50)) * expected_denominator;
            assert_eq!(
                result.gross_evsi,
                Some(Rational {
                    numerator: expected_numerator,
                    denominator: expected_denominator
                }),
                "signal quality {signal_quality}"
            );
            assert_eq!(result.selected, signal_quality != 50);
        }
    }

    #[test]
    fn fifty_case_prior_and_signal_matrix_preserves_evsi_invariants() {
        for prior_hypothesis_a in (10_u64..=90).step_by(10) {
            for signal_quality in (60_u64..=100).step_by(10) {
                let case = DecisionCase {
                    likelihood_scale: 100,
                    hypotheses: vec![
                        Hypothesis {
                            id: "cause-a".to_owned(),
                            prior_weight: prior_hypothesis_a,
                        },
                        Hypothesis {
                            id: "cause-b".to_owned(),
                            prior_weight: 100 - prior_hypothesis_a,
                        },
                    ],
                    actions: vec![
                        Action {
                            id: "ACTION_A".to_owned(),
                            loss_by_hypothesis: vec![0, 100],
                            admissible: true,
                        },
                        Action {
                            id: "ACTION_B".to_owned(),
                            loss_by_hypothesis: vec![100, 0],
                            admissible: true,
                        },
                        Action {
                            id: "NO_ACTION".to_owned(),
                            loss_by_hypothesis: vec![60, 60],
                            admissible: true,
                        },
                    ],
                };
                let measurement = Measurement {
                    id: format!("prior-{prior_hypothesis_a}-signal-{signal_quality}"),
                    authorised: true,
                    safe: true,
                    acquisition_cost: 3,
                    delay_cost: 2,
                    outcomes: vec![
                        Outcome {
                            id: "red".to_owned(),
                            likelihood_weight_by_hypothesis: vec![
                                signal_quality,
                                100 - signal_quality,
                            ],
                        },
                        Outcome {
                            id: "blue".to_owned(),
                            likelihood_weight_by_hypothesis: vec![
                                100 - signal_quality,
                                signal_quality,
                            ],
                        },
                    ],
                };

                let result =
                    &evaluate(&case, &[measurement]).expect("matrix case should validate")[0];
                let baseline = result.baseline_expected_loss.expect("baseline loss");
                let post = result
                    .expected_post_measurement_loss
                    .expect("post-measurement loss");
                let gross = result.gross_evsi.expect("gross EVSI");
                let net = result.net_evsi.expect("net EVSI");

                assert!(
                    post.numerator * baseline.denominator
                        <= baseline.numerator * post.denominator,
                    "information increased expected loss for prior {prior_hypothesis_a}, signal {signal_quality}"
                );
                assert_eq!(
                    gross.numerator,
                    baseline.numerator * 100 - post.numerator,
                    "gross EVSI mismatch for prior {prior_hypothesis_a}, signal {signal_quality}"
                );
                assert_eq!(
                    net.numerator,
                    i128::try_from(gross.numerator).unwrap()
                        - i128::try_from(5_u128 * gross.denominator).unwrap(),
                    "net EVSI cost mismatch for prior {prior_hypothesis_a}, signal {signal_quality}"
                );
                assert_eq!(result.outcome_choices.len(), 2);
                assert_eq!(
                    result
                        .outcome_choices
                        .iter()
                        .map(|choice| choice.predictive_mass)
                        .sum::<u128>(),
                    10_000
                );
            }
        }
    }

    #[test]
    fn measurement_order_does_not_change_results() {
        let case = decision_case();
        let first = discriminating_measurement("z-check");
        let second = discriminating_measurement("a-check");
        let forward = evaluate(&case, &[first.clone(), second.clone()]).expect("forward");
        let reverse = evaluate(&case, &[second, first]).expect("reverse");
        assert_eq!(forward, reverse);
    }

    #[test]
    fn action_order_does_not_change_tie_breaking_or_results() {
        let mut case = decision_case();
        let mut reversed = case.actions.clone();
        reversed.reverse();
        case.actions.reverse();
        let mut other = decision_case();
        other.actions = reversed;
        let first = evaluate(&case, &[discriminating_measurement("same")]).expect("first");
        let second = evaluate(&other, &[discriminating_measurement("same")]).expect("second");
        assert_eq!(first, second);
    }

    #[test]
    fn only_the_highest_positive_net_evsi_measurement_is_selected() {
        let mut more_expensive = discriminating_measurement("z-flow-survey");
        more_expensive.acquisition_cost = 8;
        let less_expensive = discriminating_measurement("a-meter-check");
        let results = evaluate(&decision_case(), &[more_expensive, less_expensive])
            .expect("both candidate measurements should validate");
        assert_eq!(results.len(), 2);
        assert_eq!(results[0].measurement_id, "a-meter-check");
        assert!(results[0].selected);
        assert!(!results[1].selected);
    }

    #[test]
    fn measurement_outcome_conditions_next_snapshot_exactly() {
        let case = decision_case();
        let first = discriminating_measurement("first-check");
        let posterior = condition_case_on_outcome(&case, &first, "red")
            .expect("authorised safe observation should condition the case");
        assert_eq!(
            posterior
                .hypotheses
                .iter()
                .map(|hypothesis| hypothesis.prior_weight)
                .collect::<Vec<_>>(),
            vec![9, 1]
        );

        let second = Measurement {
            id: "second-check".to_owned(),
            authorised: true,
            safe: true,
            acquisition_cost: 1,
            delay_cost: 0,
            outcomes: vec![
                Outcome {
                    id: "cause-a".to_owned(),
                    likelihood_weight_by_hypothesis: vec![100, 0],
                },
                Outcome {
                    id: "cause-b".to_owned(),
                    likelihood_weight_by_hypothesis: vec![0, 100],
                },
            ],
        };
        let ranked = evaluate(&posterior, &[second]).expect("posterior case should evaluate");
        assert_eq!(ranked[0].baseline_action_id.as_deref(), Some("ACTION_A"));
        assert_eq!(ranked[0].outcome_choices.len(), 2);
    }

    #[test]
    fn impossible_outcome_is_skipped_after_perfect_prior_observation() {
        let case = decision_case();
        let perfect = Measurement {
            id: "perfect-check".to_owned(),
            authorised: true,
            safe: true,
            acquisition_cost: 0,
            delay_cost: 0,
            outcomes: vec![
                Outcome {
                    id: "cause-a".to_owned(),
                    likelihood_weight_by_hypothesis: vec![100, 0],
                },
                Outcome {
                    id: "cause-b".to_owned(),
                    likelihood_weight_by_hypothesis: vec![0, 100],
                },
            ],
        };
        let posterior = condition_case_on_outcome(&case, &perfect, "cause-a")
            .expect("cause-a should be possible");
        assert_eq!(
            posterior
                .hypotheses
                .iter()
                .map(|hypothesis| hypothesis.prior_weight)
                .collect::<Vec<_>>(),
            vec![1, 0]
        );
        let ranked = evaluate(&posterior, std::slice::from_ref(&perfect))
            .expect("perfect measurement should evaluate against posterior");
        assert_eq!(ranked[0].outcome_choices.len(), 1);
        assert_eq!(ranked[0].outcome_choices[0].outcome_id, "cause-a");
        assert!(condition_case_on_outcome(&posterior, &perfect, "cause-b")
            .unwrap_err()
            .contains("impossible"));
    }

    #[test]
    fn unauthorised_or_unsafe_measurements_cannot_condition_a_snapshot() {
        let case = decision_case();
        let mut unauthorised = discriminating_measurement("unauthorised");
        unauthorised.authorised = false;
        assert!(condition_case_on_outcome(&case, &unauthorised, "red")
            .unwrap_err()
            .contains("unauthorised"));
        let mut unsafe_measurement = discriminating_measurement("unsafe");
        unsafe_measurement.safe = false;
        assert!(condition_case_on_outcome(&case, &unsafe_measurement, "red")
            .unwrap_err()
            .contains("unsafe"));
    }

    #[test]
    fn acquisition_and_delay_cost_can_make_positive_gross_evsi_nonpositive() {
        let mut measurement = discriminating_measurement("expensive-check");
        measurement.acquisition_cost = 50;
        measurement.delay_cost = 10;
        let result = &evaluate(&decision_case(), &[measurement]).expect("case should validate")[0];
        assert_eq!(
            result.gross_evsi,
            Some(Rational {
                numerator: 400_000,
                denominator: 10_000
            })
        );
        assert_eq!(
            result.net_evsi,
            Some(SignedRational {
                numerator: -200_000,
                denominator: 10_000
            })
        );
        assert!(!result.selected);
    }
}
