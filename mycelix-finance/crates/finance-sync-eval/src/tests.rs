use mycelix_finance_sync_graph::{
    build_settlement_graph_v1, BoundedText, Commitment32, SemanticProfileRefV1,
    SettlementGraphInputV1, SettlementGraphV1,
};
use serde::Deserialize;

use super::*;

#[derive(Deserialize)]
struct GraphFixture {
    input: SettlementGraphInputV1,
}

#[derive(Deserialize)]
struct ObservationVectorFixture {
    input: ObservationEvaluationInputV1,
    expected: ObservationVectorExpected,
}

#[derive(Deserialize)]
struct ObservationVectorExpected {
    usd_observation_commitment: String,
    eur_observation_commitment: String,
    disposition: String,
    receipt_commitment: String,
}

fn graph() -> SettlementGraphV1 {
    let fixture: GraphFixture = serde_json::from_str(include_str!(
        "../../finance-sync-graph/test-vectors/settlement-graph-v1.json"
    ))
    .expect("FIN-SYNC-001 fixture must parse");
    build_settlement_graph_v1(fixture.input).expect("FIN-SYNC-001 fixture must construct")
}

fn leg_id_for_asset(graph: &SettlementGraphV1, asset: &str) -> Commitment32 {
    graph
        .legs()
        .iter()
        .find(|leg| leg.amount().asset().as_str() == asset)
        .expect("fixture asset must exist")
        .leg_id()
}

fn profile(id: &str, byte: u8) -> SemanticProfileRefV1 {
    SemanticProfileRefV1::new(id, 1, Commitment32::from_bytes([byte; 32]))
        .expect("static profile must be valid")
}

fn observation(
    leg_id: Commitment32,
    stream: &str,
    evidence_id: &str,
    revision: u64,
    class: ObservationClassV1,
    evidence_byte: u8,
) -> LegObservationInputV1 {
    LegObservationInputV1 {
        leg_id,
        observation_stream_ref: BoundedText::new(stream).expect("valid stream"),
        evidence_id: BoundedText::new(evidence_id).expect("valid evidence id"),
        evidence_commitment: Commitment32::from_bytes([evidence_byte; 32]),
        observation_revision: revision,
        class,
        observation_profile: profile("observation:synthetic:v1", 0xab),
    }
}

fn evaluate(
    graph: &SettlementGraphV1,
    observations: Vec<LegObservationInputV1>,
) -> Result<ObservedGraphReceiptV1, EvalError> {
    evaluate_observed_graph_v1(
        graph,
        ObservationEvaluationInputV1 {
            graph_commitment: graph.graph_commitment(),
            evaluation_profile: profile("fin-sync:observation-eval:v1", 0xdd),
            evaluation_context_commitment: Commitment32::from_bytes([0xee; 32]),
            observations,
        },
    )
}

#[test]
fn frozen_vector_matches_independent_oracle() {
    let graph = graph();
    let fixture: ObservationVectorFixture = serde_json::from_str(include_str!(
        "../test-vectors/observed-graph-v1.json"
    ))
    .expect("observation vector must parse");

    let receipt = evaluate_observed_graph_v1(&graph, fixture.input)
        .expect("frozen observation vector must evaluate");

    let usd_expected = Commitment32::from_hex(&fixture.expected.usd_observation_commitment)
        .expect("valid USD expected commitment");
    let eur_expected = Commitment32::from_hex(&fixture.expected.eur_observation_commitment)
        .expect("valid EUR expected commitment");
    let expected_receipt = Commitment32::from_hex(&fixture.expected.receipt_commitment)
        .expect("valid expected receipt commitment");

    assert!(receipt.observation_commitments().contains(&usd_expected));
    assert!(receipt.observation_commitments().contains(&eur_expected));
    assert_eq!(receipt.receipt_commitment(), expected_receipt);
    assert_eq!(
        serde_json::to_value(receipt.disposition()).expect("serialize disposition"),
        serde_json::Value::String(fixture.expected.disposition)
    );
}

#[test]
fn applied_plus_unknown_preserves_unknown_instead_of_inventing_partial_failure() {
    let graph = graph();
    let usd = leg_id_for_asset(&graph, "USD:bank-a");
    let eur = leg_id_for_asset(&graph, "EUR:bank-b");

    let receipt = evaluate(
        &graph,
        vec![
            observation(
                usd,
                "stream:usd",
                "evidence:usd:applied",
                1,
                ObservationClassV1::ReportedAppliedUnqualified,
                0x11,
            ),
            observation(
                eur,
                "stream:eur",
                "evidence:eur:unknown",
                1,
                ObservationClassV1::ReportedOutcomeUnknown,
                0x22,
            ),
        ],
    )
    .expect("bounded input");

    assert_eq!(
        receipt.disposition(),
        ObservedGraphDispositionV1::UnknownEffectPossible
    );
}

#[test]
fn applied_plus_definitely_rejected_is_partial_effect_observed() {
    let graph = graph();
    let usd = leg_id_for_asset(&graph, "USD:bank-a");
    let eur = leg_id_for_asset(&graph, "EUR:bank-b");

    let receipt = evaluate(
        &graph,
        vec![
            observation(
                usd,
                "stream:usd",
                "evidence:usd:applied",
                1,
                ObservationClassV1::ReportedAppliedUnqualified,
                0x11,
            ),
            observation(
                eur,
                "stream:eur",
                "evidence:eur:rejected",
                1,
                ObservationClassV1::ReportedDefinitelyRejectedBeforeEffect,
                0x22,
            ),
        ],
    )
    .expect("bounded input");

    assert_eq!(
        receipt.disposition(),
        ObservedGraphDispositionV1::PartialEffectObserved
    );
}

#[test]
fn all_applied_remains_explicitly_unqualified() {
    let graph = graph();
    let usd = leg_id_for_asset(&graph, "USD:bank-a");
    let eur = leg_id_for_asset(&graph, "EUR:bank-b");

    let receipt = evaluate(
        &graph,
        vec![
            observation(
                usd,
                "stream:usd",
                "evidence:usd:applied",
                1,
                ObservationClassV1::ReportedAppliedUnqualified,
                0x11,
            ),
            observation(
                eur,
                "stream:eur",
                "evidence:eur:applied",
                1,
                ObservationClassV1::ReportedAppliedUnqualified,
                0x22,
            ),
        ],
    )
    .expect("bounded input");

    assert_eq!(
        receipt.disposition(),
        ObservedGraphDispositionV1::AllRequiredEffectsObservedAppliedButUnqualified
    );
}

#[test]
fn higher_revision_only_supersedes_within_its_exact_stream() {
    let graph = graph();
    let usd = leg_id_for_asset(&graph, "USD:bank-a");
    let eur = leg_id_for_asset(&graph, "EUR:bank-b");

    let receipt = evaluate(
        &graph,
        vec![
            observation(
                usd,
                "stream:usd:a",
                "evidence:usd:a:100",
                100,
                ObservationClassV1::ReportedAppliedUnqualified,
                0x11,
            ),
            observation(
                usd,
                "stream:usd:b",
                "evidence:usd:b:1",
                1,
                ObservationClassV1::ReportedRejectedUnqualified,
                0x12,
            ),
            observation(
                eur,
                "stream:eur",
                "evidence:eur:applied",
                1,
                ObservationClassV1::ReportedAppliedUnqualified,
                0x22,
            ),
        ],
    )
    .expect("bounded input");

    let usd_state = receipt
        .leg_dispositions()
        .iter()
        .find(|leg| leg.leg_id() == usd)
        .expect("USD leg frontier")
        .state();
    assert_eq!(usd_state, DerivedLegObservationV1::Conflicted);
    assert_eq!(receipt.disposition(), ObservedGraphDispositionV1::Conflicted);
}

#[test]
fn higher_revision_refines_unknown_inside_one_stream() {
    let graph = graph();
    let usd = leg_id_for_asset(&graph, "USD:bank-a");
    let eur = leg_id_for_asset(&graph, "EUR:bank-b");

    let receipt = evaluate(
        &graph,
        vec![
            observation(
                usd,
                "stream:usd",
                "evidence:usd:1",
                1,
                ObservationClassV1::ReportedOutcomeUnknown,
                0x10,
            ),
            observation(
                usd,
                "stream:usd",
                "evidence:usd:2",
                2,
                ObservationClassV1::ReportedAppliedUnqualified,
                0x11,
            ),
            observation(
                eur,
                "stream:eur",
                "evidence:eur:1",
                1,
                ObservationClassV1::ReportedAppliedUnqualified,
                0x22,
            ),
        ],
    )
    .expect("bounded input");

    assert_eq!(
        receipt.disposition(),
        ObservedGraphDispositionV1::AllRequiredEffectsObservedAppliedButUnqualified
    );
    let usd_frontier = receipt
        .leg_dispositions()
        .iter()
        .find(|leg| leg.leg_id() == usd)
        .expect("USD frontier");
    assert_eq!(usd_frontier.selected_streams()[0].selected_revision(), 2);
}

#[test]
fn same_evidence_identity_with_changed_semantics_is_conflict() {
    let graph = graph();
    let usd = leg_id_for_asset(&graph, "USD:bank-a");

    let first = observation(
        usd,
        "stream:usd",
        "evidence:stable-id",
        1,
        ObservationClassV1::ReportedAppliedUnqualified,
        0x11,
    );
    let mut changed = first.clone();
    changed.class = ObservationClassV1::ReportedRejectedUnqualified;

    let receipt = evaluate(&graph, vec![first, changed]).expect("conflict is a typed outcome");
    assert_eq!(receipt.disposition(), ObservedGraphDispositionV1::Conflicted);
}

#[test]
fn same_stream_same_revision_with_different_semantics_is_conflict() {
    let graph = graph();
    let usd = leg_id_for_asset(&graph, "USD:bank-a");

    let receipt = evaluate(
        &graph,
        vec![
            observation(
                usd,
                "stream:usd",
                "evidence:usd:a",
                7,
                ObservationClassV1::ReportedAppliedUnqualified,
                0x11,
            ),
            observation(
                usd,
                "stream:usd",
                "evidence:usd:b",
                7,
                ObservationClassV1::ReportedRejectedUnqualified,
                0x12,
            ),
        ],
    )
    .expect("conflict is a typed outcome");

    assert_eq!(receipt.disposition(), ObservedGraphDispositionV1::Conflicted);
}

#[test]
fn exact_duplicate_evidence_is_idempotent() {
    let graph = graph();
    let usd = leg_id_for_asset(&graph, "USD:bank-a");
    let eur = leg_id_for_asset(&graph, "EUR:bank-b");
    let usd_observation = observation(
        usd,
        "stream:usd",
        "evidence:usd:1",
        1,
        ObservationClassV1::ReportedAppliedUnqualified,
        0x11,
    );
    let eur_observation = observation(
        eur,
        "stream:eur",
        "evidence:eur:1",
        1,
        ObservationClassV1::ReportedOutcomeUnknown,
        0x22,
    );

    let baseline = evaluate(
        &graph,
        vec![usd_observation.clone(), eur_observation.clone()],
    )
    .expect("baseline");
    let duplicate = evaluate(
        &graph,
        vec![
            usd_observation.clone(),
            usd_observation,
            eur_observation,
        ],
    )
    .expect("duplicate replay");

    assert_eq!(baseline.receipt_commitment(), duplicate.receipt_commitment());
}

#[test]
fn input_permutation_does_not_change_receipt() {
    let graph = graph();
    let usd = leg_id_for_asset(&graph, "USD:bank-a");
    let eur = leg_id_for_asset(&graph, "EUR:bank-b");
    let observations = vec![
        observation(
            usd,
            "stream:usd:a",
            "evidence:usd:a",
            1,
            ObservationClassV1::ReportedAppliedUnqualified,
            0x11,
        ),
        observation(
            usd,
            "stream:usd:b",
            "evidence:usd:b",
            1,
            ObservationClassV1::ReportedOutcomeUnknown,
            0x12,
        ),
        observation(
            eur,
            "stream:eur",
            "evidence:eur",
            1,
            ObservationClassV1::ReportedOutcomeUnknown,
            0x22,
        ),
    ];

    let baseline = evaluate(&graph, observations.clone()).expect("baseline");
    let mut reversed = observations;
    reversed.reverse();
    let permuted = evaluate(&graph, reversed).expect("permuted");

    assert_eq!(baseline.receipt_commitment(), permuted.receipt_commitment());
}

#[test]
fn reversal_is_append_only_observation_not_erasure() {
    let graph = graph();
    let usd = leg_id_for_asset(&graph, "USD:bank-a");
    let eur = leg_id_for_asset(&graph, "EUR:bank-b");

    let receipt = evaluate(
        &graph,
        vec![
            observation(
                usd,
                "stream:usd",
                "evidence:usd:applied",
                1,
                ObservationClassV1::ReportedAppliedUnqualified,
                0x11,
            ),
            observation(
                usd,
                "stream:usd",
                "evidence:usd:reversed",
                2,
                ObservationClassV1::ReportedReversedUnqualified,
                0x12,
            ),
            observation(
                eur,
                "stream:eur",
                "evidence:eur:applied",
                1,
                ObservationClassV1::ReportedAppliedUnqualified,
                0x22,
            ),
        ],
    )
    .expect("reversal observation");

    assert_eq!(receipt.disposition(), ObservedGraphDispositionV1::ReversalObserved);
    assert_eq!(receipt.observation_commitments().len(), 3);
}

#[test]
fn unknown_leg_is_rejected() {
    let graph = graph();
    let alien = observation(
        Commitment32::from_bytes([0x99; 32]),
        "stream:alien",
        "evidence:alien",
        1,
        ObservationClassV1::ReportedOutcomeUnknown,
        0x99,
    );

    assert_eq!(evaluate(&graph, vec![alien]), Err(EvalError::UnknownLeg));
}

#[test]
fn wrong_graph_commitment_is_rejected() {
    let graph = graph();
    let result = evaluate_observed_graph_v1(
        &graph,
        ObservationEvaluationInputV1 {
            graph_commitment: Commitment32::from_bytes([0x77; 32]),
            evaluation_profile: profile("fin-sync:observation-eval:v1", 0xdd),
            evaluation_context_commitment: Commitment32::from_bytes([0xee; 32]),
            observations: Vec::new(),
        },
    );

    assert_eq!(result, Err(EvalError::GraphCommitmentMismatch));
}
