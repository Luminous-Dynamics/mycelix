// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Validated pure-strategy Nash analysis for finite normal-form games.
//!
//! This module is intentionally additive. It validates the complete payoff
//! table before computing best responses and does not replace the legacy
//! GameTheory APIs until downstream callers have been migrated.

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use super::game_theory::GameDefinition;
use super::game_theory_validation::{
    validate_game_definition, GameValidationConfig, GameValidationReport,
};

/// Absolute and relative tolerance used when comparing player payoffs.
///
/// Use zero tolerances for exact integer-valued analytical fixtures. Nonzero
/// tolerances are explicit configuration, not hidden solver constants.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PayoffTolerance {
    /// Absolute payoff tolerance in the game's declared utility unit.
    pub absolute: f64,
    /// Relative tolerance multiplied by max(|a|, |b|).
    pub relative: f64,
}

impl Default for PayoffTolerance {
    fn default() -> Self {
        Self {
            absolute: 0.0,
            relative: 0.0,
        }
    }
}

impl PayoffTolerance {
    fn valid(self) -> bool {
        self.absolute.is_finite()
            && self.relative.is_finite()
            && self.absolute >= 0.0
            && self.relative >= 0.0
    }

    fn threshold(self, a: f64, b: f64) -> f64 {
        self.absolute + self.relative * a.abs().max(b.abs())
    }
}

/// Configuration for validated pure-equilibrium analysis.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PureEquilibriumConfig {
    /// Validation and profile-enumeration limits.
    pub validation: GameValidationConfig,
    /// Explicit payoff comparison tolerance.
    pub tolerance: PayoffTolerance,
}

impl Default for PureEquilibriumConfig {
    fn default() -> Self {
        Self {
            validation: GameValidationConfig::default(),
            tolerance: PayoffTolerance::default(),
        }
    }
}

/// Why validated equilibrium analysis could not be completed.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PureEquilibriumError {
    /// The game failed structural or payoff-table completeness validation.
    InvalidGame {
        /// Complete validation report describing the invalid game.
        report: GameValidationReport,
    },
    /// The requested payoff tolerance is negative or non-finite.
    InvalidTolerance {
        /// The invalid tolerance supplied by the caller.
        tolerance: PayoffTolerance,
    },
}

/// Whether a complete pure-strategy search found an equilibrium.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PureEquilibriumStatus {
    /// One or more pure-strategy Nash equilibria were found.
    Found,
    /// Every legal profile has at least one profitable unilateral deviation.
    NoPureEquilibrium,
}

/// Best-response evidence for one player at one candidate profile.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BestResponseEvidence {
    /// Player identifier.
    pub player_id: String,
    /// Strategy played in the candidate profile.
    pub played_strategy: String,
    /// Payoff received under the played strategy.
    pub played_payoff: f64,
    /// Highest payoff available through a legal unilateral deviation.
    pub best_response_payoff: f64,
    /// Best-response payoff minus played payoff, floored at zero.
    pub unilateral_regret: f64,
    /// Strategies tied for best response under the configured tolerance.
    pub best_response_strategies: Vec<String>,
    /// Whether the played strategy is a best response under the tolerance.
    pub played_strategy_is_best_response: bool,
}

/// A pure Nash equilibrium and its per-player best-response evidence.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PureNashCertificate {
    /// Ordered strategy profile, one strategy per player.
    pub strategy_profile: Vec<String>,
    /// Ordered payoff vector, one payoff per player.
    pub payoffs: Vec<f64>,
    /// Best-response evidence for every player.
    pub best_responses: Vec<BestResponseEvidence>,
    /// Largest unilateral regret across all players (within configured tolerance).
    pub maximum_unilateral_regret: f64,
}

/// Result of a complete pure-strategy analysis over a validated game.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PureEquilibriumAnalysis {
    /// Search outcome, distinguishing no pure equilibrium from invalid input.
    pub status: PureEquilibriumStatus,
    /// Number of legal pure profiles evaluated.
    pub profiles_evaluated: usize,
    /// Tolerance used for every payoff comparison.
    pub tolerance: PayoffTolerance,
    /// All pure-strategy Nash equilibria found, in deterministic profile order.
    pub equilibria: Vec<PureNashCertificate>,
}

/// Analyze every legal pure profile after requiring a complete, valid payoff table.
///
/// This routine:
/// - rejects malformed/incomplete games before analysis;
/// - respects each player's declared legal strategy set;
/// - checks every legal unilateral deviation for every player;
/// - exposes the configured payoff tolerance and each player's regret; and
/// - distinguishes a valid game with no pure equilibrium from invalid input.
///
/// It does not compute mixed-strategy, Bayesian, extensive-form, repeated-game,
/// coalition-proof, or evolutionary equilibria.
pub fn analyze_pure_nash_equilibria(
    game: &GameDefinition,
    config: PureEquilibriumConfig,
) -> Result<PureEquilibriumAnalysis, PureEquilibriumError> {
    if !config.tolerance.valid() {
        return Err(PureEquilibriumError::InvalidTolerance {
            tolerance: config.tolerance,
        });
    }

    let report = validate_game_definition(game, config.validation);
    if !report.is_valid || !report.profile_enumeration_complete {
        return Err(PureEquilibriumError::InvalidGame { report });
    }

    let legal_strategy_ids: Vec<Vec<String>> = game
        .players
        .iter()
        .map(|player| {
            game.strategies
                .iter()
                .filter(|strategy| strategy.applicable_types.contains(&player.player_type))
                .map(|strategy| strategy.id.clone())
                .collect()
        })
        .collect();

    let payoffs_by_profile: HashMap<Vec<String>, Vec<f64>> = game
        .payoffs
        .iter()
        .map(|entry| (entry.strategy_profile.clone(), entry.payoffs.clone()))
        .collect();

    let mut profiles = Vec::with_capacity(report.expected_profile_count.unwrap_or(0));
    enumerate_profiles(
        &legal_strategy_ids,
        0,
        &mut Vec::with_capacity(game.players.len()),
        &mut profiles,
    );

    let mut equilibria = Vec::new();

    for profile in &profiles {
        let Some(played_payoffs) = payoffs_by_profile.get(profile) else {
            // Validation has already proven completeness. Keep this boundary
            // defensive in case the implementation is changed independently.
            let defensive_report = validate_game_definition(game, config.validation);
            return Err(PureEquilibriumError::InvalidGame {
                report: defensive_report,
            });
        };

        let mut best_responses = Vec::with_capacity(game.players.len());
        let mut profile_is_equilibrium = true;
        let mut maximum_unilateral_regret: f64 = 0.0;

        for (player_index, player) in game.players.iter().enumerate() {
            let played_payoff = played_payoffs[player_index];
            let mut best_payoff = f64::NEG_INFINITY;
            let mut deviation_payoffs = Vec::with_capacity(legal_strategy_ids[player_index].len());

            for strategy_id in &legal_strategy_ids[player_index] {
                let mut deviation_profile = profile.clone();
                deviation_profile[player_index] = strategy_id.clone();

                let deviation_payoff = payoffs_by_profile
                    .get(&deviation_profile)
                    .map_or(f64::NEG_INFINITY, |payoffs| payoffs[player_index]);

                best_payoff = best_payoff.max(deviation_payoff);
                deviation_payoffs.push((strategy_id.clone(), deviation_payoff));
            }

            let comparison_tolerance =
                config.tolerance.threshold(best_payoff, played_payoff);
            let unilateral_regret = (best_payoff - played_payoff).max(0.0);
            let played_strategy_is_best_response =
                unilateral_regret <= comparison_tolerance;

            if !played_strategy_is_best_response {
                profile_is_equilibrium = false;
            }
            maximum_unilateral_regret =
                maximum_unilateral_regret.max(unilateral_regret);

            let best_response_strategies = deviation_payoffs
                .into_iter()
                .filter_map(|(strategy_id, payoff)| {
                    let tie_tolerance = config.tolerance.threshold(best_payoff, payoff);
                    ((best_payoff - payoff).max(0.0) <= tie_tolerance)
                        .then_some(strategy_id)
                })
                .collect();

            best_responses.push(BestResponseEvidence {
                player_id: player.id.clone(),
                played_strategy: profile[player_index].clone(),
                played_payoff,
                best_response_payoff: best_payoff,
                unilateral_regret,
                best_response_strategies,
                played_strategy_is_best_response,
            });
        }

        if profile_is_equilibrium {
            equilibria.push(PureNashCertificate {
                strategy_profile: profile.clone(),
                payoffs: played_payoffs.clone(),
                best_responses,
                maximum_unilateral_regret,
            });
        }
    }

    let status = if equilibria.is_empty() {
        PureEquilibriumStatus::NoPureEquilibrium
    } else {
        PureEquilibriumStatus::Found
    };

    Ok(PureEquilibriumAnalysis {
        status,
        profiles_evaluated: profiles.len(),
        tolerance: config.tolerance,
        equilibria,
    })
}

fn enumerate_profiles(
    legal_strategy_ids: &[Vec<String>],
    player_index: usize,
    current: &mut Vec<String>,
    output: &mut Vec<Vec<String>>,
) {
    if player_index == legal_strategy_ids.len() {
        output.push(current.clone());
        return;
    }

    for strategy_id in &legal_strategy_ids[player_index] {
        current.push(strategy_id.clone());
        enumerate_profiles(legal_strategy_ids, player_index + 1, current, output);
        current.pop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agentic::game_theory::{
        ActionCondition, ActionType, GameDefinition, PayoffEntry, Player, PlayerType, Strategy,
        StrategyAction,
    };

    fn strategy(id: &str) -> Strategy {
        Strategy {
            id: id.to_string(),
            name: id.to_string(),
            description: format!("{id} strategy"),
            actions: vec![StrategyAction {
                action_type: if id == "cooperate" {
                    ActionType::Cooperate
                } else {
                    ActionType::Defect
                },
                probability: 1.0,
                conditions: vec![ActionCondition::Always],
            }],
            applicable_types: vec![PlayerType::Rational],
        }
    }

    fn game(payoffs: Vec<PayoffEntry>) -> GameDefinition {
        GameDefinition {
            name: "canonical game theory fixture".to_string(),
            description: "Hand-checked finite normal-form game".to_string(),
            players: vec![
                Player {
                    id: "player-1".to_string(),
                    player_type: PlayerType::Rational,
                    initial_trust: 0.5,
                    initial_kredit: 100,
                },
                Player {
                    id: "player-2".to_string(),
                    player_type: PlayerType::Rational,
                    initial_trust: 0.5,
                    initial_kredit: 100,
                },
            ],
            strategies: vec![strategy("cooperate"), strategy("defect")],
            payoffs,
            rounds: 1,
            discount_factor: 1.0,
        }
    }

    fn payoff(a: &str, b: &str, x: f64, y: f64) -> PayoffEntry {
        PayoffEntry {
            strategy_profile: vec![a.to_string(), b.to_string()],
            payoffs: vec![x, y],
        }
    }

    fn exact_config() -> PureEquilibriumConfig {
        PureEquilibriumConfig::default()
    }

    #[test]
    fn prisoners_dilemma_has_unique_defection_equilibrium() {
        let game = game(vec![
            payoff("cooperate", "cooperate", 3.0, 3.0),
            payoff("cooperate", "defect", 0.0, 5.0),
            payoff("defect", "cooperate", 5.0, 0.0),
            payoff("defect", "defect", 1.0, 1.0),
        ]);

        let result = analyze_pure_nash_equilibria(&game, exact_config()).unwrap();

        assert_eq!(result.status, PureEquilibriumStatus::Found);
        assert_eq!(result.profiles_evaluated, 4);
        assert_eq!(result.equilibria.len(), 1);
        assert_eq!(
            result.equilibria[0].strategy_profile,
            vec!["defect".to_string(), "defect".to_string()]
        );
        assert!(result.equilibria[0]
            .best_responses
            .iter()
            .all(|proof| proof.played_strategy_is_best_response));
    }

    #[test]
    fn matching_pennies_has_no_pure_equilibrium() {
        let game = game(vec![
            payoff("cooperate", "cooperate", 1.0, -1.0),
            payoff("cooperate", "defect", -1.0, 1.0),
            payoff("defect", "cooperate", -1.0, 1.0),
            payoff("defect", "defect", 1.0, -1.0),
        ]);

        let result = analyze_pure_nash_equilibria(&game, exact_config()).unwrap();

        assert_eq!(result.status, PureEquilibriumStatus::NoPureEquilibrium);
        assert_eq!(result.profiles_evaluated, 4);
        assert!(result.equilibria.is_empty());
    }

    #[test]
    fn stag_hunt_has_two_pure_equilibria() {
        let game = game(vec![
            payoff("cooperate", "cooperate", 4.0, 4.0),
            payoff("cooperate", "defect", 0.0, 3.0),
            payoff("defect", "cooperate", 3.0, 0.0),
            payoff("defect", "defect", 2.0, 2.0),
        ]);

        let result = analyze_pure_nash_equilibria(&game, exact_config()).unwrap();

        assert_eq!(result.status, PureEquilibriumStatus::Found);
        assert_eq!(result.equilibria.len(), 2);
        assert!(result.equilibria.iter().any(|equilibrium| {
            equilibrium.strategy_profile == vec!["cooperate", "cooperate"]
        }));
        assert!(result.equilibria.iter().any(|equilibrium| {
            equilibrium.strategy_profile == vec!["defect", "defect"]
        }));
    }

    #[test]
    fn asymmetric_player_legal_sets_have_unique_equilibrium() {
        let malicious_strategy = |id: &str| Strategy {
            id: id.to_string(),
            name: id.to_string(),
            description: format!("{id} strategy"),
            actions: vec![StrategyAction {
                action_type: ActionType::Defect,
                probability: 1.0,
                conditions: vec![ActionCondition::Always],
            }],
            applicable_types: vec![PlayerType::Malicious],
        };
        let mut game = game(vec![
            PayoffEntry {
                strategy_profile: vec!["provide".into(), "attack".into()],
                payoffs: vec![0.0, 3.0],
            },
            PayoffEntry {
                strategy_profile: vec!["provide".into(), "abstain".into()],
                payoffs: vec![4.0, 0.0],
            },
            PayoffEntry {
                strategy_profile: vec!["withhold".into(), "attack".into()],
                payoffs: vec![1.0, 2.0],
            },
            PayoffEntry {
                strategy_profile: vec!["withhold".into(), "abstain".into()],
                payoffs: vec![2.0, 1.0],
            },
        ]);
        game.players[0].id = "provider".to_string();
        game.players[1].id = "attacker".to_string();
        game.players[1].player_type = PlayerType::Malicious;
        game.strategies = vec![
            Strategy {
                id: "provide".to_string(),
                name: "provide".to_string(),
                description: "provide".to_string(),
                actions: vec![StrategyAction {
                    action_type: ActionType::Cooperate,
                    probability: 1.0,
                    conditions: vec![ActionCondition::Always],
                }],
                applicable_types: vec![PlayerType::Rational],
            },
            Strategy {
                id: "withhold".to_string(),
                name: "withhold".to_string(),
                description: "withhold".to_string(),
                actions: vec![StrategyAction {
                    action_type: ActionType::Defect,
                    probability: 1.0,
                    conditions: vec![ActionCondition::Always],
                }],
                applicable_types: vec![PlayerType::Rational],
            },
            malicious_strategy("attack"),
            malicious_strategy("abstain"),
        ];

        let result = analyze_pure_nash_equilibria(&game, exact_config()).unwrap();

        assert_eq!(result.status, PureEquilibriumStatus::Found);
        assert_eq!(result.profiles_evaluated, 4);
        assert_eq!(result.equilibria.len(), 1);
        assert_eq!(
            result.equilibria[0].strategy_profile,
            vec!["withhold".to_string(), "attack".to_string()]
        );
    }

    #[test]
    fn incomplete_payoff_table_returns_validation_error() {
        let game = game(vec![
            payoff("cooperate", "cooperate", 3.0, 3.0),
            payoff("cooperate", "defect", 0.0, 5.0),
            payoff("defect", "cooperate", 5.0, 0.0),
        ]);

        let error = analyze_pure_nash_equilibria(&game, exact_config()).unwrap_err();

        assert!(matches!(
            error,
            PureEquilibriumError::InvalidGame { .. }
        ));
    }

    #[test]
    fn player_permutation_preserves_equilibrium_under_matching_payoff_permutation() {
        let original = game(vec![
            payoff("cooperate", "cooperate", 3.0, 3.0),
            payoff("cooperate", "defect", 0.0, 5.0),
            payoff("defect", "cooperate", 5.0, 0.0),
            payoff("defect", "defect", 1.0, 1.0),
        ]);
        let original_result =
            analyze_pure_nash_equilibria(&original, exact_config()).unwrap();

        let mut swapped = original.clone();
        swapped.players.swap(0, 1);
        for entry in &mut swapped.payoffs {
            entry.strategy_profile.swap(0, 1);
            entry.payoffs.swap(0, 1);
        }
        let swapped_result =
            analyze_pure_nash_equilibria(&swapped, exact_config()).unwrap();

        assert_eq!(original_result.status, swapped_result.status);
        assert_eq!(original_result.equilibria.len(), swapped_result.equilibria.len());

        let original_equilibria: Vec<(Vec<String>, Vec<f64>)> = original_result
            .equilibria
            .iter()
            .map(|equilibrium| (
                equilibrium.strategy_profile.clone(),
                equilibrium.payoffs.clone(),
            ))
            .collect();
        let swapped_equilibria: Vec<(Vec<String>, Vec<f64>)> = swapped_result
            .equilibria
            .iter()
            .map(|equilibrium| (
                equilibrium.strategy_profile.clone(),
                equilibrium.payoffs.clone(),
            ))
            .collect();
        assert_eq!(original_equilibria, swapped_equilibria);
    }

    #[test]
    fn strategy_declaration_order_does_not_change_equilibrium_set() {
        let mut original = game(vec![
            payoff("cooperate", "cooperate", 3.0, 3.0),
            payoff("cooperate", "defect", 0.0, 5.0),
            payoff("defect", "cooperate", 5.0, 0.0),
            payoff("defect", "defect", 1.0, 1.0),
        ]);
        let original_result =
            analyze_pure_nash_equilibria(&original, exact_config()).unwrap();

        original.strategies.reverse();
        let reversed_result =
            analyze_pure_nash_equilibria(&original, exact_config()).unwrap();

        let mut original_set: Vec<(Vec<String>, Vec<f64>)> = original_result
            .equilibria
            .iter()
            .map(|equilibrium| (
                equilibrium.strategy_profile.clone(),
                equilibrium.payoffs.clone(),
            ))
            .collect();
        let mut reversed_set: Vec<(Vec<String>, Vec<f64>)> = reversed_result
            .equilibria
            .iter()
            .map(|equilibrium| (
                equilibrium.strategy_profile.clone(),
                equilibrium.payoffs.clone(),
            ))
            .collect();
        original_set.sort();
        reversed_set.sort();
        assert_eq!(original_set, reversed_set);
    }

    #[test]
    fn invalid_tolerance_is_rejected_before_analysis() {
        let game = game(vec![
            payoff("cooperate", "cooperate", 3.0, 3.0),
            payoff("cooperate", "defect", 0.0, 5.0),
            payoff("defect", "cooperate", 5.0, 0.0),
            payoff("defect", "defect", 1.0, 1.0),
        ]);
        let config = PureEquilibriumConfig {
            validation: GameValidationConfig::default(),
            tolerance: PayoffTolerance {
                absolute: f64::NAN,
                relative: 0.0,
            },
        };

        assert!(matches!(
            analyze_pure_nash_equilibria(&game, config),
            Err(PureEquilibriumError::InvalidTolerance { .. })
        ));
    }

    #[test]
    fn payoff_tolerance_is_explicit_and_reported() {
        let game = game(vec![
            payoff("cooperate", "cooperate", 1.0, 1.0),
            payoff("cooperate", "defect", 1.000_000_5, 1.0),
            payoff("defect", "cooperate", 1.0, 1.000_000_5),
            payoff("defect", "defect", 1.0, 1.0),
        ]);
        let config = PureEquilibriumConfig {
            validation: GameValidationConfig::default(),
            tolerance: PayoffTolerance {
                absolute: 0.000_001,
                relative: 0.0,
            },
        };

        let result = analyze_pure_nash_equilibria(&game, config).unwrap();

        assert_eq!(result.tolerance, config.tolerance);
        assert_eq!(result.status, PureEquilibriumStatus::Found);
    }
}
