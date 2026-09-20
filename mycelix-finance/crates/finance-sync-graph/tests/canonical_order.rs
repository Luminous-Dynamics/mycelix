use mycelix_finance_exact::{AssetAmount, AssetId};
use mycelix_finance_sync_graph::{
    build_settlement_graph_v1, BoundedText, Commitment32, CoordinationGroupClassV1,
    CoordinationGroupSpecV1, DependencySpecV1, SemanticProfileRefV1,
    SettlementGraphInputV1, SettlementLegRoleV1, SettlementLegSpecV1,
};

fn text(value: &str) -> BoundedText {
    BoundedText::new(value).expect("static bounded text")
}

fn profile(id: &str, byte: u8) -> SemanticProfileRefV1 {
    SemanticProfileRefV1::new(id, 1, Commitment32::from_bytes([byte; 32]))
        .expect("static profile")
}

fn payment(alias: &str, asset: &str, amount: u64, byte: u8) -> SettlementLegSpecV1 {
    SettlementLegSpecV1 {
        alias: text(alias),
        adapter_profile: profile(&format!("adapter:{alias}:v1"), byte),
        rail: text("SYNTHETIC"),
        network: text(&format!("network:{alias}")),
        role: SettlementLegRoleV1::Payment,
        source_subject: text(&format!("source:{alias}")),
        destination_subject: text(&format!("destination:{alias}")),
        amount: AssetAmount::new(amount, AssetId::new(asset).expect("static asset")),
        asset_unit_profile: profile(&format!("asset-profile:{alias}:v1"), byte.wrapping_add(1)),
        required_finality_profile: profile(
            &format!("finality:{alias}:v1"),
            byte.wrapping_add(2),
        ),
        semantic_idempotency_ref: text(&format!("idem:{alias}")),
        purpose_profile: Some(profile(
            &format!("purpose:{alias}:v1"),
            byte.wrapping_add(3),
        )),
        delivery_asset_subject: None,
    }
}

fn three_leg_saga() -> SettlementGraphInputV1 {
    SettlementGraphInputV1 {
        economic_effect_commitment: Commitment32::from_bytes([0x11; 32]),
        graph_profile: profile("fin-sync:saga-order:v1", 0x21),
        temporal_profile: None,
        legs: vec![
            payment("a", "A", 10, 0x31),
            payment("b", "B", 20, 0x41),
            payment("c", "C", 30, 0x51),
        ],
        dependencies: vec![
            DependencySpecV1::Requires {
                leg_alias: text("c"),
                prerequisite_alias: text("b"),
            },
            DependencySpecV1::ConditionalOnEvidence {
                leg_alias: text("c"),
                predicate_profile: profile("predicate:c:v1", 0x61),
            },
            DependencySpecV1::Before {
                before_alias: text("a"),
                after_alias: text("b"),
            },
        ],
        coordination_groups: vec![CoordinationGroupSpecV1 {
            alias: text("saga"),
            class: CoordinationGroupClassV1::Saga,
            coordination_profile: profile("sync:saga-order:v1", 0x71),
            member_aliases: vec![text("a"), text("b"), text("c")],
        }],
    }
}

#[test]
fn dependency_variant_permutation_is_canonical() {
    let baseline = build_settlement_graph_v1(three_leg_saga()).expect("baseline graph");

    let mut reversed = three_leg_saga();
    reversed.dependencies.reverse();
    reversed.legs.reverse();
    reversed.coordination_groups[0].member_aliases.reverse();
    let reversed = build_settlement_graph_v1(reversed).expect("reversed graph");

    assert_eq!(baseline.graph_commitment(), reversed.graph_commitment());
}

#[test]
fn dependency_semantic_mutation_changes_graph_identity() {
    let baseline = build_settlement_graph_v1(three_leg_saga()).expect("baseline graph");

    let mut mutated = three_leg_saga();
    mutated.dependencies[0] = DependencySpecV1::Requires {
        leg_alias: text("b"),
        prerequisite_alias: text("a"),
    };
    let mutated = build_settlement_graph_v1(mutated).expect("mutated graph");

    assert_ne!(baseline.graph_commitment(), mutated.graph_commitment());
}
