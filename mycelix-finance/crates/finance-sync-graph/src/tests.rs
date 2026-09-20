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

fn text(value: &str) -> BoundedText {
    BoundedText::new(value).expect("static bounded text")
}

fn profile(id: &str, byte: u8) -> SemanticProfileRefV1 {
    SemanticProfileRefV1::new(id, 1, Commitment32::from_bytes([byte; 32]))
        .expect("static semantic profile")
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
    let graph = build_settlement_graph_v1(fixture.input).expect("frozen graph fixture");

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
    assert!(graph
        .legs()
        .iter()
        .all(|leg| leg.role() == SettlementLegRoleV1::Payment));
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
}

#[test]
fn construction_aliases_are_not_authority_bytes() {
    let original = build_settlement_graph_v1(input()).expect("baseline graph");
    let mut renamed = input();
    renamed.legs[0].alias = text("leg-alpha");
    renamed.legs[1].alias = text("leg-beta");
    match &mut renamed.dependencies[0] {
        DependencySpecV1::ConditionalOnEvidence { leg_alias, .. } => {
            *leg_alias = text("leg-alpha");
        }
        _ => panic!("fixture dependency class changed"),
    }
    let group = &mut renamed.coordination_groups[0];
    group.alias = text("construction-group-only");
    group.member_aliases = vec![text("leg-beta"), text("leg-alpha")];
    let renamed = build_settlement_graph_v1(renamed).expect("renamed graph");
    assert_eq!(original.graph_commitment(), renamed.graph_commitment());
}

#[test]
fn effect_significant_mutations_change_identity() {
    let original = build_settlement_graph_v1(input()).expect("baseline graph");

    let mut amount = input();
    let old = &amount.legs[0].amount;
    amount.legs[0].amount = mycelix_finance_exact::AssetAmount::new(
        old.atomic_units() + 1,
        old.asset().clone(),
    );
    let amount = build_settlement_graph_v1(amount).expect("amount mutation");
    assert_ne!(original.graph_commitment(), amount.graph_commitment());

    let mut effect = input();
    effect.economic_effect_commitment = Commitment32::from_bytes([0x12; 32]);
    let effect = build_settlement_graph_v1(effect).expect("effect mutation");
    assert_ne!(original.graph_commitment(), effect.graph_commitment());

    let mut graph_profile = input();
    graph_profile.graph_profile = SemanticProfileRefV1::new(
        "fin-sync:pvp:v1",
        2,
        Commitment32::from_bytes([0x2d; 32]),
    )
    .expect("valid profile");
    let graph_profile = build_settlement_graph_v1(graph_profile).expect("profile mutation");
    assert_ne!(original.graph_commitment(), graph_profile.graph_commitment());
}

#[test]
fn pvp_requires_two_payment_roles() {
    let mut invalid = input();
    invalid.legs[1].role = SettlementLegRoleV1::Delivery;
    invalid.legs[1].delivery_asset_subject = Some(text("security:example"));
    assert_eq!(
        build_settlement_graph_v1(invalid),
        Err(GraphError::InvalidCoordinationRoleComposition)
    );
}

#[test]
fn dvp_requires_one_payment_and_one_delivery() {
    let mut valid = input();
    valid.coordination_groups[0].class = CoordinationGroupClassV1::Dvp;
    valid.coordination_groups[0].coordination_profile = profile("sync:dvp:synthetic:v1", 0xcd);
    valid.legs[1].role = SettlementLegRoleV1::Delivery;
    valid.legs[1].delivery_asset_subject = Some(text("security:example-share"));
    assert!(build_settlement_graph_v1(valid).is_ok());

    let mut invalid = input();
    invalid.coordination_groups[0].class = CoordinationGroupClassV1::Dvp;
    assert_eq!(
        build_settlement_graph_v1(invalid),
        Err(GraphError::InvalidCoordinationRoleComposition)
    );
}

#[test]
fn role_local_semantics_fail_closed() {
    let mut payment_with_delivery = input();
    payment_with_delivery.legs[0].delivery_asset_subject = Some(text("asset:unexpected"));
    assert_eq!(
        build_settlement_graph_v1(payment_with_delivery),
        Err(GraphError::InvalidLegRoleSemantics)
    );

    let mut delivery_without_subject = input();
    delivery_without_subject.legs[0].role = SettlementLegRoleV1::Delivery;
    assert_eq!(
        build_settlement_graph_v1(delivery_without_subject),
        Err(GraphError::InvalidLegRoleSemantics)
    );

    let mut auxiliary_without_purpose = input();
    auxiliary_without_purpose.legs[0].role = SettlementLegRoleV1::Auxiliary;
    auxiliary_without_purpose.legs[0].purpose_profile = None;
    assert_eq!(
        build_settlement_graph_v1(auxiliary_without_purpose),
        Err(GraphError::InvalidLegRoleSemantics)
    );
}

#[test]
fn duplicate_semantics_and_replay_identity_are_rejected() {
    let mut duplicate_leg = input();
    let mut copied = duplicate_leg.legs[0].clone();
    copied.alias = text("duplicate-visible-alias");
    duplicate_leg.legs.push(copied);
    assert_eq!(
        build_settlement_graph_v1(duplicate_leg),
        Err(GraphError::DuplicateSemanticLeg)
    );

    let mut duplicate_dependency = input();
    duplicate_dependency
        .dependencies
        .push(duplicate_dependency.dependencies[0].clone());
    assert_eq!(
        build_settlement_graph_v1(duplicate_dependency),
        Err(GraphError::DuplicateDependency)
    );

    let mut duplicate_idempotency = input();
    duplicate_idempotency.legs[1].semantic_idempotency_ref =
        duplicate_idempotency.legs[0].semantic_idempotency_ref.clone();
    assert_eq!(
        build_settlement_graph_v1(duplicate_idempotency),
        Err(GraphError::DuplicateSemanticIdempotencyRef)
    );
}

#[test]
fn graph_must_be_connected_and_dependencies_must_resolve() {
    let mut disconnected = input();
    disconnected.dependencies.clear();
    disconnected.coordination_groups.clear();
    assert_eq!(
        build_settlement_graph_v1(disconnected),
        Err(GraphError::DisconnectedGraph)
    );

    let mut unknown = input();
    unknown.coordination_groups.clear();
    unknown.dependencies = vec![DependencySpecV1::Before {
        before_alias: text("usd-leg"),
        after_alias: text("missing-leg"),
    }];
    assert_eq!(
        build_settlement_graph_v1(unknown),
        Err(GraphError::UnknownLegAlias)
    );
}

#[test]
fn residual_cycles_and_fake_intragroup_ordering_are_rejected() {
    let mut cyclic = input();
    cyclic.coordination_groups.clear();
    cyclic.dependencies = vec![
        DependencySpecV1::Requires {
            leg_alias: text("usd-leg"),
            prerequisite_alias: text("eur-leg"),
        },
        DependencySpecV1::Requires {
            leg_alias: text("eur-leg"),
            prerequisite_alias: text("usd-leg"),
        },
    ];
    assert_eq!(
        build_settlement_graph_v1(cyclic),
        Err(GraphError::DependencyCycle)
    );

    let mut intragroup = input();
    intragroup.dependencies.push(DependencySpecV1::Before {
        before_alias: text("usd-leg"),
        after_alias: text("eur-leg"),
    });
    assert_eq!(
        build_settlement_graph_v1(intragroup),
        Err(GraphError::DependencyInsideStrongGroup)
    );
}

#[test]
fn group_membership_is_exclusive_in_v1() {
    let mut invalid = input();
    invalid.coordination_groups.push(CoordinationGroupSpecV1 {
        alias: text("second-group"),
        class: CoordinationGroupClassV1::Saga,
        coordination_profile: profile("sync:saga:synthetic:v1", 0xcd),
        member_aliases: vec![text("usd-leg"), text("eur-leg")],
    });
    assert_eq!(
        build_settlement_graph_v1(invalid),
        Err(GraphError::LegInMultipleGroups)
    );
}

#[test]
fn pvp_and_dvp_cardinality_is_exactly_two() {
    let mut invalid = input();
    invalid.coordination_groups[0].member_aliases = vec![text("usd-leg")];
    assert_eq!(
        build_settlement_graph_v1(invalid),
        Err(GraphError::InvalidGroupSize)
    );
}

#[test]
fn zero_and_single_leg_graphs_are_rejected() {
    let mut zero_amount = input();
    let asset = zero_amount.legs[0].amount.asset().clone();
    zero_amount.legs[0].amount = mycelix_finance_exact::AssetAmount::new(0, asset);
    assert_eq!(
        build_settlement_graph_v1(zero_amount),
        Err(GraphError::ZeroAmount)
    );

    let mut single = input();
    single.legs.truncate(1);
    single.dependencies.clear();
    single.coordination_groups.clear();
    assert_eq!(
        build_settlement_graph_v1(single),
        Err(GraphError::TooFewLegs)
    );
}

#[test]
fn predicate_changes_graph_not_leg_identity() {
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
fn unknown_json_fields_and_invalid_commitments_fail_closed() {
    let raw = include_str!("../test-vectors/settlement-graph-v1.json");
    let mut value: serde_json::Value = serde_json::from_str(raw).expect("valid fixture json");
    value["input"]["legs"][0]["settled"] = serde_json::Value::Bool(true);
    let encoded = serde_json::to_string(&value["input"]).expect("serialize fixture input");
    assert!(serde_json::from_str::<SettlementGraphInputV1>(&encoded).is_err());

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

    #[test]
    fn construction_aliases_remain_non_authoritative(
        left_suffix in 0_u32..10_000,
        right_suffix in 0_u32..10_000,
    ) {
        prop_assume!(left_suffix != right_suffix);
        let baseline = build_settlement_graph_v1(input()).expect("baseline graph");
        let mut candidate = input();
        let left = text(&format!("left-{left_suffix}"));
        let right = text(&format!("right-{right_suffix}"));
        candidate.legs[0].alias = left.clone();
        candidate.legs[1].alias = right.clone();
        match &mut candidate.dependencies[0] {
            DependencySpecV1::ConditionalOnEvidence { leg_alias, .. } => *leg_alias = left.clone(),
            _ => unreachable!("fixture dependency class"),
        }
        candidate.coordination_groups[0].member_aliases = vec![right, left];
        let candidate = build_settlement_graph_v1(candidate).expect("renamed graph");
        prop_assert_eq!(baseline.graph_commitment(), candidate.graph_commitment());
    }
}
