use proptest::prelude::*;
use serde::Deserialize;

use super::*;

#[derive(Deserialize)]
struct VectorFixture {
    input: SettlementGraphInputV1,
    expected: ExpectedVector,
}

#[derive(Deserialize)]
struct ExpectedVector {
    usd_leg_id: Commitment32,
    eur_leg_id: Commitment32,
    group_id: Commitment32,
    graph_commitment: Commitment32,
}

fn fixture() -> VectorFixture {
    serde_json::from_str(include_str!("../test-vectors/settlement-graph-v1.json"))
        .expect("frozen settlement-graph vector must parse")
}

fn input() -> SettlementGraphInputV1 {
    fixture().input
}

fn leg_id_for_asset(graph: &SettlementGraphV1, asset: &str) -> Commitment32 {
    graph
        .legs()
        .iter()
        .find(|leg| leg.amount().asset().as_str() == asset)
        .expect("fixture asset must exist")
        .leg_id()
}

#[test]
fn frozen_vector_matches_independent_oracle() {
    let fixture = fixture();
    let graph =
        build_settlement_graph_v1(fixture.input).expect("frozen graph fixture must qualify");

    assert_eq!(
        leg_id_for_asset(&graph, "USD:bank-a"),
        fixture.expected.usd_leg_id
    );
    assert_eq!(
        leg_id_for_asset(&graph, "EUR:bank-b"),
        fixture.expected.eur_leg_id
    );
    assert_eq!(graph.coordination_groups().len(), 1);
    assert_eq!(
        graph.coordination_groups()[0].group_id(),
        fixture.expected.group_id
    );
    assert_eq!(graph.graph_commitment(), fixture.expected.graph_commitment);
    assert_eq!(graph.graph_id(), fixture.expected.graph_commitment);
}

#[test]
fn semantic_set_permutations_do_not_change_identity() {
    let original = build_settlement_graph_v1(input()).expect("baseline graph");

    let mut permuted = input();
    permuted.legs.reverse();
    permuted.dependencies.reverse();
    permuted.coordination_groups.reverse();
    for group in &mut permuted.coordination_groups {
        group.member_aliases.reverse();
    }

    let permuted = build_settlement_graph_v1(permuted).expect("permuted graph");
    assert_eq!(original.graph_commitment(), permuted.graph_commitment());
    assert_eq!(
        original
            .legs()
            .iter()
            .map(SettlementLegV1::leg_id)
            .collect::<Vec<_>>(),
        permuted
            .legs()
            .iter()
            .map(SettlementLegV1::leg_id)
            .collect::<Vec<_>>()
    );
}

#[test]
fn construction_aliases_are_not_authority_bytes() {
    let original = build_settlement_graph_v1(input()).expect("baseline graph");

    let mut renamed = input();
    renamed.legs[0].alias = BoundedText::new("leg-alpha").expect("valid alias");
    renamed.legs[1].alias = BoundedText::new("leg-beta").expect("valid alias");

    match &mut renamed.dependencies[0] {
        DependencySpecV1::ConditionalOnEvidence { leg_alias, .. } => {
            *leg_alias = BoundedText::new("leg-alpha").expect("valid alias");
        }
        _ => panic!("fixture dependency class changed"),
    }

    let group = &mut renamed.coordination_groups[0];
    group.alias = BoundedText::new("construction-group-only").expect("valid alias");
    group.member_aliases = vec![
        BoundedText::new("leg-beta").expect("valid alias"),
        BoundedText::new("leg-alpha").expect("valid alias"),
    ];

    let renamed = build_settlement_graph_v1(renamed).expect("renamed graph");
    assert_eq!(original.graph_commitment(), renamed.graph_commitment());
}

#[test]
fn amount_mutation_changes_leg_and_graph_identity() {
    let original = build_settlement_graph_v1(input()).expect("baseline graph");

    let mut mutated = input();
    let amount = &mut mutated.legs[0].amount;
    *amount = mycelix_finance_exact::AssetAmount::new(
        amount.atomic_units() + 1,
        amount.asset().clone(),
    );

    let mutated = build_settlement_graph_v1(mutated).expect("mutated graph");
    assert_ne!(
        leg_id_for_asset(&original, "USD:bank-a"),
        leg_id_for_asset(&mutated, "USD:bank-a")
    );
    assert_ne!(original.graph_commitment(), mutated.graph_commitment());
}

#[test]
fn economic_effect_mutation_changes_every_leg_and_graph_identity() {
    let original = build_settlement_graph_v1(input()).expect("baseline graph");

    let mut mutated = input();
    mutated.economic_effect_commitment = Commitment32::from_bytes([0x12; 32]);
    let mutated = build_settlement_graph_v1(mutated).expect("mutated graph");

    assert_ne!(original.graph_commitment(), mutated.graph_commitment());
    for asset in ["USD:bank-a", "EUR:bank-b"] {
        assert_ne!(
            leg_id_for_asset(&original, asset),
            leg_id_for_asset(&mutated, asset)
        );
    }
}

#[test]
fn graph_profile_mutation_changes_leg_and_graph_identity() {
    let original = build_settlement_graph_v1(input()).expect("baseline graph");

    let mut mutated = input();
    mutated.graph_profile = SemanticProfileRefV1::new(
        "fin-sync:pvp:v1",
        2,
        Commitment32::from_bytes([0x2d; 32]),
    )
    .expect("valid profile");
    let mutated = build_settlement_graph_v1(mutated).expect("mutated graph");

    assert_ne!(original.graph_commitment(), mutated.graph_commitment());
    assert_ne!(
        leg_id_for_asset(&original, "USD:bank-a"),
        leg_id_for_asset(&mutated, "USD:bank-a")
    );
}

#[test]
fn duplicate_semantic_leg_under_an_alias_is_rejected() {
    let mut duplicate = input();
    let mut copied = duplicate.legs[0].clone();
    copied.alias = BoundedText::new("duplicate-visible-alias").expect("valid alias");
    duplicate.legs.push(copied);

    assert_eq!(
        build_settlement_graph_v1(duplicate),
        Err(GraphError::DuplicateSemanticLeg)
    );
}

#[test]
fn duplicate_dependency_is_rejected() {
    let mut duplicate = input();
    duplicate.dependencies.push(duplicate.dependencies[0].clone());

    assert_eq!(
        build_settlement_graph_v1(duplicate),
        Err(GraphError::DuplicateDependency)
    );
}

#[test]
fn duplicate_semantic_idempotency_reference_is_rejected() {
    let mut invalid = input();
    invalid.legs[1].semantic_idempotency_ref =
        invalid.legs[0].semantic_idempotency_ref.clone();

    assert_eq!(
        build_settlement_graph_v1(invalid),
        Err(GraphError::DuplicateSemanticIdempotencyRef)
    );
}

#[test]
fn unrelated_leg_bag_is_not_a_settlement_graph() {
    let mut invalid = input();
    invalid.dependencies.clear();
    invalid.coordination_groups.clear();

    assert_eq!(
        build_settlement_graph_v1(invalid),
        Err(GraphError::DisconnectedGraph)
    );
}

#[test]
fn unknown_dependency_alias_is_rejected() {
    let mut unknown = input();
    unknown.dependencies = vec![DependencySpecV1::Before {
        before_alias: BoundedText::new("usd-leg").expect("valid alias"),
        after_alias: BoundedText::new("missing-leg").expect("valid alias"),
    }];

    assert_eq!(
        build_settlement_graph_v1(unknown),
        Err(GraphError::UnknownLegAlias)
    );
}

#[test]
fn residual_dependency_cycle_is_rejected() {
    let mut cyclic = input();
    cyclic.coordination_groups.clear();
    cyclic.dependencies = vec![
        DependencySpecV1::Requires {
            leg_alias: BoundedText::new("usd-leg").expect("valid alias"),
            prerequisite_alias: BoundedText::new("eur-leg").expect("valid alias"),
        },
        DependencySpecV1::Requires {
            leg_alias: BoundedText::new("eur-leg").expect("valid alias"),
            prerequisite_alias: BoundedText::new("usd-leg").expect("valid alias"),
        },
    ];

    assert_eq!(
        build_settlement_graph_v1(cyclic),
        Err(GraphError::DependencyCycle)
    );
}

#[test]
fn ordinary_dependency_inside_strong_group_is_rejected() {
    let mut invalid = input();
    invalid.dependencies.push(DependencySpecV1::Before {
        before_alias: BoundedText::new("usd-leg").expect("valid alias"),
        after_alias: BoundedText::new("eur-leg").expect("valid alias"),
    });

    assert_eq!(
        build_settlement_graph_v1(invalid),
        Err(GraphError::DependencyInsideStrongGroup)
    );
}

#[test]
fn one_leg_cannot_belong_to_two_coordination_groups_in_v1() {
    let mut invalid = input();
    invalid.coordination_groups.push(CoordinationGroupSpecV1 {
        alias: BoundedText::new("second-group").expect("valid alias"),
        class: CoordinationGroupClassV1::Saga,
        coordination_profile: SemanticProfileRefV1::new(
            "sync:saga:synthetic:v1",
            1,
            Commitment32::from_bytes([0xcd; 32]),
        )
        .expect("valid profile"),
        member_aliases: vec![
            BoundedText::new("usd-leg").expect("valid alias"),
            BoundedText::new("eur-leg").expect("valid alias"),
        ],
    });

    assert_eq!(
        build_settlement_graph_v1(invalid),
        Err(GraphError::LegInMultipleGroups)
    );
}

#[test]
fn pvp_requires_exactly_two_members_in_v1() {
    let mut invalid = input();
    invalid.coordination_groups[0].member_aliases =
        vec![BoundedText::new("usd-leg").expect("valid alias")];

    assert_eq!(
        build_settlement_graph_v1(invalid),
        Err(GraphError::InvalidGroupSize)
    );
}

#[test]
fn zero_amount_leg_is_rejected() {
    let mut invalid = input();
    let asset = invalid.legs[0].amount.asset().clone();
    invalid.legs[0].amount = mycelix_finance_exact::AssetAmount::new(0, asset);

    assert_eq!(
        build_settlement_graph_v1(invalid),
        Err(GraphError::ZeroAmount)
    );
}

#[test]
fn single_leg_is_out_of_scope_for_fin_sync_v1() {
    let mut invalid = input();
    invalid.legs.truncate(1);
    invalid.dependencies.clear();
    invalid.coordination_groups.clear();

    assert_eq!(
        build_settlement_graph_v1(invalid),
        Err(GraphError::TooFewLegs)
    );
}

#[test]
fn conditional_predicate_changes_graph_but_not_settlement_leg_identity() {
    let original = build_settlement_graph_v1(input()).expect("baseline graph");

    let mut mutated = input();
    match &mut mutated.dependencies[0] {
        DependencySpecV1::ConditionalOnEvidence {
            predicate_profile, ..
        } => {
            *predicate_profile = SemanticProfileRefV1::new(
                "compliance:synthetic:v2",
                2,
                Commitment32::from_bytes([0xbc; 32]),
            )
            .expect("valid profile");
        }
        _ => panic!("fixture dependency class changed"),
    }
    let mutated = build_settlement_graph_v1(mutated).expect("mutated graph");

    assert_eq!(
        leg_id_for_asset(&original, "USD:bank-a"),
        leg_id_for_asset(&mutated, "USD:bank-a")
    );
    assert_ne!(original.graph_commitment(), mutated.graph_commitment());
}

#[test]
fn unknown_json_fields_cannot_smuggle_runtime_state_into_graph_input() {
    let raw = include_str!("../test-vectors/settlement-graph-v1.json");
    let mut value: serde_json::Value = serde_json::from_str(raw).expect("valid fixture json");
    value["input"]["legs"][0]["settled"] = serde_json::Value::Bool(true);

    let encoded = serde_json::to_string(&value["input"]).expect("serialize fixture input");
    assert!(serde_json::from_str::<SettlementGraphInputV1>(&encoded).is_err());
}

#[test]
fn invalid_commitment_hex_is_rejected() {
    assert_eq!(
        Commitment32::from_hex("abc"),
        Err(GraphError::InvalidCommitmentHex)
    );
    assert_eq!(
        Commitment32::from_hex(&"z".repeat(64)),
        Err(GraphError::InvalidCommitmentHex)
    );
}

proptest! {
    #[test]
    fn nonzero_amount_mutation_is_commitment_sensitive(
        left in 1_u64..10_000_000,
        right in 1_u64..10_000_000,
    ) {
        prop_assume!(left != right);

        let mut first_input = input();
        let asset = first_input.legs[0].amount.asset().clone();
        first_input.legs[0].amount =
            mycelix_finance_exact::AssetAmount::new(left, asset.clone());

        let mut second_input = input();
        second_input.legs[0].amount =
            mycelix_finance_exact::AssetAmount::new(right, asset);

        let first = build_settlement_graph_v1(first_input).expect("bounded graph");
        let second = build_settlement_graph_v1(second_input).expect("bounded graph");

        prop_assert_ne!(
            leg_id_for_asset(&first, "USD:bank-a"),
            leg_id_for_asset(&second, "USD:bank-a")
        );
        prop_assert_ne!(first.graph_commitment(), second.graph_commitment());
    }
}
