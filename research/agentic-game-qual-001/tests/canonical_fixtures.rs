use std::collections::HashMap;

use mycelix_agentic_game_qual_harness::agentic::game_theory::{
    ActionCondition, ActionType, GameDefinition, PayoffEntry, Player, PlayerType, Strategy,
    StrategyAction,
};
use mycelix_agentic_game_qual_harness::agentic::game_theory_verified::{
    analyze_pure_nash_equilibria, PureEquilibriumConfig, PureEquilibriumStatus,
};
use serde::Deserialize;

const FIXTURE_JSON: &str = include_str!("../fixtures/canonical-finite-games-v0.1.json");

#[derive(Debug, Deserialize)]
struct FixtureManifest {
    fixtures: Vec<GameFixture>,
}

#[derive(Debug, Deserialize)]
struct GameFixture {
    id: String,
    name: String,
    players: Vec<PlayerFixture>,
    strategies: Vec<StrategyFixture>,
    payoffs: Vec<PayoffFixture>,
    expected_pure_equilibria: Vec<ExpectedEquilibrium>,
}

#[derive(Debug, Deserialize)]
struct PlayerFixture {
    id: String,
    #[serde(rename = "type")]
    player_type: String,
}

#[derive(Debug, Deserialize)]
struct StrategyFixture {
    id: String,
    legal_for: Vec<String>,
}

#[derive(Debug, Deserialize)]
struct PayoffFixture {
    profile: Vec<String>,
    values: Vec<f64>,
}

#[derive(Debug, Deserialize)]
struct ExpectedEquilibrium {
    profile: Vec<String>,
    payoffs: Vec<f64>,
}

fn parse_player_type(value: &str) -> PlayerType {
    match value {
        "Honest" => PlayerType::Honest,
        "Rational" => PlayerType::Rational,
        "Malicious" => PlayerType::Malicious,
        "Byzantine" => PlayerType::Byzantine,
        other => panic!("unsupported fixture player type: {other}"),
    }
}

fn build_game(fixture: &GameFixture) -> GameDefinition {
    let players = fixture
        .players
        .iter()
        .map(|player| Player {
            id: player.id.clone(),
            player_type: parse_player_type(&player.player_type),
            initial_trust: 0.5,
            initial_kredit: 100,
        })
        .collect();

    let strategies = fixture
        .strategies
        .iter()
        .map(|strategy| Strategy {
            id: strategy.id.clone(),
            name: strategy.id.clone(),
            description: format!("Canonical fixture strategy {}", strategy.id),
            actions: vec![StrategyAction {
                action_type: ActionType::Custom {
                    name: strategy.id.clone(),
                    params: HashMap::new(),
                },
                probability: 1.0,
                conditions: vec![ActionCondition::Always],
            }],
            applicable_types: strategy
                .legal_for
                .iter()
                .map(|player_type| parse_player_type(player_type))
                .collect(),
        })
        .collect();

    let payoffs = fixture
        .payoffs
        .iter()
        .map(|entry| PayoffEntry {
            strategy_profile: entry.profile.clone(),
            payoffs: entry.values.clone(),
        })
        .collect();

    GameDefinition {
        name: fixture.name.clone(),
        description: format!("Canonical machine-readable fixture {}", fixture.id),
        players,
        strategies,
        payoffs,
        rounds: 1,
        discount_factor: 1.0,
    }
}

#[test]
fn canonical_json_fixtures_match_the_verified_solver() {
    let manifest: FixtureManifest =
        serde_json::from_str(FIXTURE_JSON).expect("canonical fixture JSON must parse");

    assert_eq!(manifest.fixtures.len(), 4);

    for fixture in &manifest.fixtures {
        let game = build_game(fixture);
        let analysis = analyze_pure_nash_equilibria(&game, PureEquilibriumConfig::default())
            .unwrap_or_else(|error| panic!("{} failed validation: {error:?}", fixture.id));

        let expected_status = if fixture.expected_pure_equilibria.is_empty() {
            PureEquilibriumStatus::NoPureEquilibrium
        } else {
            PureEquilibriumStatus::Found
        };
        assert_eq!(
            analysis.status, expected_status,
            "unexpected equilibrium status for {}",
            fixture.id
        );

        let actual: Vec<(Vec<String>, Vec<f64>)> = analysis
            .equilibria
            .iter()
            .map(|equilibrium| {
                (
                    equilibrium.strategy_profile.clone(),
                    equilibrium.payoffs.clone(),
                )
            })
            .collect();
        let expected: Vec<(Vec<String>, Vec<f64>)> = fixture
            .expected_pure_equilibria
            .iter()
            .map(|equilibrium| {
                (
                    equilibrium.profile.clone(),
                    equilibrium.payoffs.clone(),
                )
            })
            .collect();

        assert_eq!(actual, expected, "wrong pure equilibria for {}", fixture.id);
    }
}
