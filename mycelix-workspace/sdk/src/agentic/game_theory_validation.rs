// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Fail-closed validation for finite normal-form game definitions.
//!
//! This module validates the structure and completeness of a finite payoff
//! table before an analysis engine consumes it. It does not itself compute
//! equilibria or establish that a modeled utility represents human welfare.
//! In particular, validation of a payoff table must not be presented as proof
//! of incentive compatibility, Sybil resistance, collusion resistance,
//! fairness, legitimacy, or real-world enforceability.

use std::collections::{HashMap, HashSet};

use serde::{Deserialize, Serialize};

use super::game_theory::{ActionCondition, ActionType, GameDefinition, Player, Strategy};

/// Bounds the amount of payoff-profile enumeration performed by validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GameValidationConfig {
    /// Maximum number of legal pure strategy profiles to enumerate.
    ///
    /// This protects callers from accidentally requesting exponential work
    /// on a large game. Exceeding the cap is reported as an explicit issue.
    pub max_profiles: usize,
}

impl Default for GameValidationConfig {
    fn default() -> Self {
        Self {
            max_profiles: 100_000,
        }
    }
}

/// A structural or completeness problem found in a game definition.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum GameValidationIssue {
    /// The game name is empty.
    EmptyGameName,
    /// A game must contain at least one player.
    NoPlayers,
    /// A game must declare at least one strategy.
    NoStrategies,
    /// A player identifier is empty.
    EmptyPlayerId {
        /// Zero-based position of the player in the game definition.
        player_index: usize,
    },
    /// A player identifier appears more than once.
    DuplicatePlayerId {
        /// Identifier shared by multiple player records.
        player_id: String,
    },
    /// A player's initial trust is outside [0, 1] or non-finite.
    InvalidInitialTrust {
        /// Identifier of the player with the invalid trust value.
        player_id: String,
    },
    /// A strategy identifier is empty.
    EmptyStrategyId {
        /// Zero-based position of the strategy in the game definition.
        strategy_index: usize,
    },
    /// A strategy identifier appears more than once.
    DuplicateStrategyId {
        /// Identifier shared by multiple strategy records.
        strategy_id: String,
    },
    /// A strategy has no action definitions.
    StrategyHasNoActions {
        /// Identifier of the strategy with no actions.
        strategy_id: String,
    },
    /// An action probability is outside [0, 1] or non-finite.
    InvalidActionProbability {
        /// Identifier of the strategy containing the action.
        strategy_id: String,
        /// Zero-based position of the invalid action in its strategy.
        action_index: usize,
    },
    /// An action contains a non-finite numeric parameter or an empty required name.
    InvalidActionParameter {
        /// Identifier of the strategy containing the action.
        strategy_id: String,
        /// Zero-based position of the invalid action in its strategy.
        action_index: usize,
        /// Name of the invalid parameter.
        parameter: String,
    },
    /// A condition contains an invalid numeric threshold.
    InvalidConditionParameter {
        /// Identifier of the strategy containing the action.
        strategy_id: String,
        /// Zero-based position of the action in its strategy.
        action_index: usize,
        /// Zero-based position of the invalid condition in the action.
        condition_index: usize,
    },
    /// No declared strategy is legal for a player type.
    PlayerHasNoApplicableStrategies {
        /// Identifier of the player without a legal strategy.
        player_id: String,
    },
    /// The repeated-game round count is zero.
    ZeroRounds,
    /// The discount factor is non-finite or outside [0, 1].
    InvalidDiscountFactor,
    /// A payoff entry has the wrong number of player actions.
    WrongProfileArity {
        /// Zero-based position of the payoff entry.
        entry_index: usize,
        /// Number of players expected in the profile.
        expected: usize,
        /// Number of strategy identifiers actually supplied.
        actual: usize,
    },
    /// A payoff entry refers to an unknown strategy.
    UnknownStrategyInProfile {
        /// Zero-based position of the payoff entry.
        entry_index: usize,
        /// Zero-based player position using the unknown strategy.
        player_index: usize,
        /// Unknown strategy identifier.
        strategy_id: String,
    },
    /// A payoff entry assigns a strategy to a player type for which it is not declared.
    InapplicableStrategyInProfile {
        /// Zero-based position of the payoff entry.
        entry_index: usize,
        /// Zero-based player position with the inapplicable strategy.
        player_index: usize,
        /// Strategy identifier not declared for this player's type.
        strategy_id: String,
    },
    /// Two payoff entries define the same ordered strategy profile.
    DuplicatePayoffProfile {
        /// Ordered strategy identifiers that occur more than once.
        profile: Vec<String>,
    },
    /// A payoff vector does not have exactly one value per player.
    WrongPayoffArity {
        /// Zero-based position of the payoff entry.
        entry_index: usize,
        /// Number of player payoffs expected.
        expected: usize,
        /// Number of payoffs actually supplied.
        actual: usize,
    },
    /// A payoff is NaN or infinite.
    NonFinitePayoff {
        /// Zero-based position of the payoff entry.
        entry_index: usize,
        /// Zero-based player position with the non-finite payoff.
        player_index: usize,
    },
    /// The legal profile count overflowed usize while being calculated.
    ProfileCountOverflow,
    /// The legal profile space exceeds the configured validation bound.
    ProfileSpaceTooLarge {
        /// Total legal profile count.
        expected: usize,
        /// Configured maximum profile count.
        maximum: usize,
    },
    /// One or more legal profiles have no valid payoff entry.
    MissingPayoffProfiles {
        /// Number of legal profiles without a valid payoff entry.
        count: usize,
        /// Up to eight missing profiles, in player order, for diagnosis.
        examples: Vec<Vec<String>>,
    },
}

/// Result of validating a finite normal-form payoff table.
///
/// `is_valid` is true only when the definition is structurally valid and every
/// legal pure-strategy profile has exactly one finite payoff vector.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GameValidationReport {
    /// Whether all structural and completeness checks passed.
    pub is_valid: bool,
    /// Number of legal pure-strategy profiles, when safely calculable.
    pub expected_profile_count: Option<usize>,
    /// Number of unique, well-formed payoff entries.
    pub observed_valid_profile_count: usize,
    /// Whether the entire legal profile space was enumerated.
    pub profile_enumeration_complete: bool,
    /// Validation findings. An empty list is required for `is_valid = true`.
    pub issues: Vec<GameValidationIssue>,
}

/// Validate a finite normal-form game definition.
///
/// The validator:
/// - uses each player's declared `PlayerType` and each strategy's
///   `applicable_types` to construct the legal profile space;
/// - rejects malformed, duplicate, unknown, inapplicable, or non-finite rows;
/// - requires exactly one valid payoff row for every legal pure profile; and
/// - bounds enumeration to avoid unbounded exponential work.
///
/// A missing payoff row is never interpreted as a zero payoff. This function
/// is a preflight validator only: existing equilibrium and incentive-analysis
/// APIs are not automatically gated by it until their callers are migrated.
pub fn validate_game_definition(
    game: &GameDefinition,
    config: GameValidationConfig,
) -> GameValidationReport {
    let mut issues = Vec::new();

    if game.name.trim().is_empty() {
        issues.push(GameValidationIssue::EmptyGameName);
    }
    if game.players.is_empty() {
        issues.push(GameValidationIssue::NoPlayers);
    }
    if game.strategies.is_empty() {
        issues.push(GameValidationIssue::NoStrategies);
    }
    if game.rounds == 0 {
        issues.push(GameValidationIssue::ZeroRounds);
    }
    if !game.discount_factor.is_finite()
        || !(0.0..=1.0).contains(&game.discount_factor)
    {
        issues.push(GameValidationIssue::InvalidDiscountFactor);
    }

    let mut player_ids = HashSet::new();
    for (index, player) in game.players.iter().enumerate() {
        validate_player(player, index, &mut player_ids, &mut issues);
    }

    let mut strategy_ids = HashSet::new();
    let mut strategies_by_id: HashMap<&str, &Strategy> = HashMap::new();
    for (index, strategy) in game.strategies.iter().enumerate() {
        if strategy.id.trim().is_empty() {
            issues.push(GameValidationIssue::EmptyStrategyId {
                strategy_index: index,
            });
        } else if !strategy_ids.insert(strategy.id.as_str()) {
            issues.push(GameValidationIssue::DuplicateStrategyId {
                strategy_id: strategy.id.clone(),
            });
        }
        strategies_by_id.insert(strategy.id.as_str(), strategy);
        validate_strategy(strategy, &mut issues);
    }

    let legal_strategy_ids: Vec<Vec<String>> = game
        .players
        .iter()
        .map(|player| {
            let legal: Vec<String> = game
                .strategies
                .iter()
                .filter(|strategy| strategy.applicable_types.contains(&player.player_type))
                .map(|strategy| strategy.id.clone())
                .collect();
            if legal.is_empty() {
                issues.push(GameValidationIssue::PlayerHasNoApplicableStrategies {
                    player_id: player.id.clone(),
                });
            }
            legal
        })
        .collect();

    let mut seen_profiles: HashSet<Vec<String>> = HashSet::new();
    let mut valid_profiles: HashSet<Vec<String>> = HashSet::new();

    for (entry_index, entry) in game.payoffs.iter().enumerate() {
        let mut row_is_valid = true;

        if entry.strategy_profile.len() != game.players.len() {
            issues.push(GameValidationIssue::WrongProfileArity {
                entry_index,
                expected: game.players.len(),
                actual: entry.strategy_profile.len(),
            });
            row_is_valid = false;
        }

        if !seen_profiles.insert(entry.strategy_profile.clone()) {
            issues.push(GameValidationIssue::DuplicatePayoffProfile {
                profile: entry.strategy_profile.clone(),
            });
            row_is_valid = false;
        }

        for (player_index, strategy_id) in entry.strategy_profile.iter().enumerate() {
            let Some(strategy) = strategies_by_id.get(strategy_id.as_str()) else {
                issues.push(GameValidationIssue::UnknownStrategyInProfile {
                    entry_index,
                    player_index,
                    strategy_id: strategy_id.clone(),
                });
                row_is_valid = false;
                continue;
            };

            if let Some(player) = game.players.get(player_index) {
                if !strategy.applicable_types.contains(&player.player_type) {
                    issues.push(GameValidationIssue::InapplicableStrategyInProfile {
                        entry_index,
                        player_index,
                        strategy_id: strategy_id.clone(),
                    });
                    row_is_valid = false;
                }
            }
        }

        if entry.payoffs.len() != game.players.len() {
            issues.push(GameValidationIssue::WrongPayoffArity {
                entry_index,
                expected: game.players.len(),
                actual: entry.payoffs.len(),
            });
            row_is_valid = false;
        }

        for (player_index, payoff) in entry.payoffs.iter().enumerate() {
            if !payoff.is_finite() {
                issues.push(GameValidationIssue::NonFinitePayoff {
                    entry_index,
                    player_index,
                });
                row_is_valid = false;
            }
        }

        if row_is_valid {
            valid_profiles.insert(entry.strategy_profile.clone());
        }
    }

    let has_empty_legal_set = legal_strategy_ids.iter().any(Vec::is_empty);
    let expected_profile_count = if game.players.is_empty() || has_empty_legal_set {
        None
    } else {
        let mut count = 1usize;
        let mut overflowed = false;
        for strategies in &legal_strategy_ids {
            match count.checked_mul(strategies.len()) {
                Some(next) => count = next,
                None => {
                    overflowed = true;
                    break;
                }
            }
        }
        if overflowed {
            issues.push(GameValidationIssue::ProfileCountOverflow);
            None
        } else {
            Some(count)
        }
    };

    let mut profile_enumeration_complete = false;

    if let Some(expected) = expected_profile_count {
        if expected > config.max_profiles {
            issues.push(GameValidationIssue::ProfileSpaceTooLarge {
                expected,
                maximum: config.max_profiles,
            });
        } else {
            let mut all_profiles = Vec::with_capacity(expected);
            enumerate_profiles(
                &legal_strategy_ids,
                0,
                &mut Vec::with_capacity(game.players.len()),
                &mut all_profiles,
            );
            profile_enumeration_complete = true;

            let mut missing_examples = Vec::new();
            let mut missing_count = 0usize;
            for profile in all_profiles {
                if !valid_profiles.contains(&profile) {
                    missing_count += 1;
                    if missing_examples.len() < 8 {
                        missing_examples.push(profile);
                    }
                }
            }

            if missing_count > 0 {
                issues.push(GameValidationIssue::MissingPayoffProfiles {
                    count: missing_count,
                    examples: missing_examples,
                });
            }
        }
    }

    GameValidationReport {
        is_valid: issues.is_empty(),
        expected_profile_count,
        observed_valid_profile_count: valid_profiles.len(),
        profile_enumeration_complete,
        issues,
    }
}

fn validate_player(
    player: &Player,
    index: usize,
    seen_ids: &mut HashSet<String>,
    issues: &mut Vec<GameValidationIssue>,
) {
    if player.id.trim().is_empty() {
        issues.push(GameValidationIssue::EmptyPlayerId {
            player_index: index,
        });
    } else if !seen_ids.insert(player.id.clone()) {
        issues.push(GameValidationIssue::DuplicatePlayerId {
            player_id: player.id.clone(),
        });
    }

    if !player.initial_trust.is_finite() || !(0.0..=1.0).contains(&player.initial_trust) {
        issues.push(GameValidationIssue::InvalidInitialTrust {
            player_id: player.id.clone(),
        });
    }
}

fn validate_strategy(strategy: &Strategy, issues: &mut Vec<GameValidationIssue>) {
    if strategy.actions.is_empty() {
        issues.push(GameValidationIssue::StrategyHasNoActions {
            strategy_id: strategy.id.clone(),
        });
        return;
    }

    for (action_index, action) in strategy.actions.iter().enumerate() {
        if !action.probability.is_finite() || !(0.0..=1.0).contains(&action.probability) {
            issues.push(GameValidationIssue::InvalidActionProbability {
                strategy_id: strategy.id.clone(),
                action_index,
            });
        }

        let invalid_parameter = match &action.action_type {
            ActionType::VoteStrategic { target_outcome } => {
                target_outcome.trim().is_empty().then_some("target_outcome")
            }
            ActionType::AttestFalse { inflated_trust } => {
                (!inflated_trust.is_finite()).then_some("inflated_trust")
            }
            ActionType::ManipulateTrust { target_delta } => {
                (!target_delta.is_finite()).then_some("target_delta")
            }
            ActionType::Custom { name, params } => {
                if name.trim().is_empty() {
                    Some("name")
                } else if params.values().any(|value| !value.is_finite()) {
                    Some("params")
                } else {
                    None
                }
            }
            _ => None,
        };

        if let Some(parameter) = invalid_parameter {
            issues.push(GameValidationIssue::InvalidActionParameter {
                strategy_id: strategy.id.clone(),
                action_index,
                parameter: parameter.to_string(),
            });
        }

        for (condition_index, condition) in action.conditions.iter().enumerate() {
            let valid = match condition {
                ActionCondition::TrustAbove(value)
                | ActionCondition::TrustBelow(value)
                | ActionCondition::OthersCooperating(value) => {
                    value.is_finite() && (0.0..=1.0).contains(value)
                }
                ActionCondition::KreditAbove(_) | ActionCondition::RoundAfter(_) | ActionCondition::Always => true,
            };

            if !valid {
                issues.push(GameValidationIssue::InvalidConditionParameter {
                    strategy_id: strategy.id.clone(),
                    action_index,
                    condition_index,
                });
            }
        }
    }
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

    fn complete_two_player_game() -> GameDefinition {
        GameDefinition {
            name: "complete two-player fixture".to_string(),
            description: "Complete 2x2 finite normal-form fixture".to_string(),
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
            payoffs: vec![
                PayoffEntry {
                    strategy_profile: vec!["cooperate".into(), "cooperate".into()],
                    payoffs: vec![3.0, 3.0],
                },
                PayoffEntry {
                    strategy_profile: vec!["cooperate".into(), "defect".into()],
                    payoffs: vec![0.0, 5.0],
                },
                PayoffEntry {
                    strategy_profile: vec!["defect".into(), "cooperate".into()],
                    payoffs: vec![5.0, 0.0],
                },
                PayoffEntry {
                    strategy_profile: vec!["defect".into(), "defect".into()],
                    payoffs: vec![1.0, 1.0],
                },
            ],
            rounds: 1,
            discount_factor: 1.0,
        }
    }

    #[test]
    fn complete_payoff_table_is_accepted() {
        let report = validate_game_definition(
            &complete_two_player_game(),
            GameValidationConfig::default(),
        );

        assert!(report.is_valid, "{:?}", report.issues);
        assert_eq!(report.expected_profile_count, Some(4));
        assert_eq!(report.observed_valid_profile_count, 4);
        assert!(report.profile_enumeration_complete);
    }

    #[test]
    fn missing_profile_is_not_silently_assigned_zero() {
        let mut game = complete_two_player_game();
        game.payoffs.pop();

        let report = validate_game_definition(&game, GameValidationConfig::default());

        assert!(!report.is_valid);
        assert!(report.issues.iter().any(|issue| matches!(
            issue,
            GameValidationIssue::MissingPayoffProfiles { count: 1, .. }
        )));
    }

    #[test]
    fn duplicate_profile_is_rejected() {
        let mut game = complete_two_player_game();
        game.payoffs.push(game.payoffs[0].clone());

        let report = validate_game_definition(&game, GameValidationConfig::default());

        assert!(!report.is_valid);
        assert!(report.issues.iter().any(|issue| matches!(
            issue,
            GameValidationIssue::DuplicatePayoffProfile { .. }
        )));
    }

    #[test]
    fn inapplicable_strategy_is_rejected() {
        let mut game = complete_two_player_game();
        game.players[1].player_type = PlayerType::Malicious;

        let report = validate_game_definition(&game, GameValidationConfig::default());

        assert!(!report.is_valid);
        assert!(report.issues.iter().any(|issue| matches!(
            issue,
            GameValidationIssue::PlayerHasNoApplicableStrategies { .. }
        )));
    }

    #[test]
    fn asymmetric_legal_strategy_sets_can_be_complete() {
        let mut game = complete_two_player_game();
        game.players[1].player_type = PlayerType::Malicious;
        let malicious_only = Strategy {
            id: "malicious_only".to_string(),
            name: "Malicious-only action".to_string(),
            description: "A strategy restricted to Malicious players".to_string(),
            actions: vec![StrategyAction {
                action_type: ActionType::Defect,
                probability: 1.0,
                conditions: vec![ActionCondition::Always],
            }],
            applicable_types: vec![PlayerType::Malicious],
        };
        game.strategies.push(malicious_only);
        game.payoffs = vec![
            PayoffEntry {
                strategy_profile: vec!["cooperate".into(), "malicious_only".into()],
                payoffs: vec![2.0, 3.0],
            },
            PayoffEntry {
                strategy_profile: vec!["defect".into(), "malicious_only".into()],
                payoffs: vec![1.0, 4.0],
            },
        ];

        let report = validate_game_definition(&game, GameValidationConfig::default());

        assert!(report.is_valid, "{:?}", report.issues);
        assert_eq!(report.expected_profile_count, Some(2));
        assert_eq!(report.observed_valid_profile_count, 2);
    }

    #[test]
    fn legal_profile_count_overflow_is_rejected_without_enumeration() {
        let mut game = complete_two_player_game();
        game.players = (0..70)
            .map(|index| Player {
                id: format!("player-{index}"),
                player_type: PlayerType::Rational,
                initial_trust: 0.5,
                initial_kredit: 100,
            })
            .collect();
        game.payoffs.clear();

        let report = validate_game_definition(&game, GameValidationConfig::default());

        assert!(!report.is_valid);
        assert_eq!(report.expected_profile_count, None);
        assert!(!report.profile_enumeration_complete);
        assert!(report
            .issues
            .contains(&GameValidationIssue::ProfileCountOverflow));
    }

    #[test]
    fn non_finite_payoffs_are_rejected() {
        let mut game = complete_two_player_game();
        game.payoffs[0].payoffs[0] = f64::NAN;

        let report = validate_game_definition(&game, GameValidationConfig::default());

        assert!(!report.is_valid);
        assert!(report.issues.iter().any(|issue| matches!(
            issue,
            GameValidationIssue::NonFinitePayoff {
                entry_index: 0,
                player_index: 0
            }
        )));
    }

    #[test]
    fn invalid_probabilities_and_discount_factors_are_rejected() {
        let mut game = complete_two_player_game();
        game.strategies[0].actions[0].probability = 1.5;
        game.discount_factor = f64::NAN;

        let report = validate_game_definition(&game, GameValidationConfig::default());

        assert!(!report.is_valid);
        assert!(report.issues.iter().any(|issue| matches!(
            issue,
            GameValidationIssue::InvalidActionProbability { .. }
        )));
        assert!(report
            .issues
            .contains(&GameValidationIssue::InvalidDiscountFactor));
    }

    #[test]
    fn profile_space_limit_is_explicit() {
        let report = validate_game_definition(
            &complete_two_player_game(),
            GameValidationConfig { max_profiles: 3 },
        );

        assert!(!report.is_valid);
        assert!(!report.profile_enumeration_complete);
        assert!(report.issues.contains(&GameValidationIssue::ProfileSpaceTooLarge {
            expected: 4,
            maximum: 3,
        }));
    }

    #[test]
    fn wrong_payoff_arity_is_rejected() {
        let mut game = complete_two_player_game();
        game.payoffs[0].payoffs.pop();

        let report = validate_game_definition(&game, GameValidationConfig::default());

        assert!(!report.is_valid);
        assert!(report.issues.iter().any(|issue| matches!(
            issue,
            GameValidationIssue::WrongPayoffArity {
                entry_index: 0,
                expected: 2,
                actual: 1
            }
        )));
    }
}
