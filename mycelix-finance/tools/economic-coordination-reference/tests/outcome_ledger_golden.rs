//! Independent golden-case checks for the procurement outcome ledger.
//!
//! This integration test intentionally uses a frozen event trace and literal
//! expected metrics. It does not call the production unit-test fixture or reuse
//! internal projection helpers.

use economic_coordination_reference::outcomes::{
    AppendDisposition, IssuerRole, OutcomeEvent, OutcomeEventKind, OutcomeLedger,
    OutcomeState, TimeInterval,
};
use economic_coordination_reference::{EvidenceOrigin, EvidenceStatus};

fn event(
    event_id: &str,
    claim_id: &str,
    role: IssuerRole,
    occurred_at: u64,
    observed_at: u64,
    kind: OutcomeEventKind,
) -> OutcomeEvent {
    OutcomeEvent {
        event_id: event_id.to_owned(),
        claim_id: claim_id.to_owned(),
        case_id: "golden-case-001".to_owned(),
        commitment_id: "golden-commitment-001".to_owned(),
        issuer_role: role,
        occurred_at,
        observed_at,
        evidence_status: EvidenceStatus::Verified,
        origin: EvidenceOrigin::LiveDomain,
        source_uri: format!("https://fixture.invalid/claims/{claim_id}"),
        source_payload_digest: format!("fixture-digest-{claim_id}"),
        supersedes_event_id: None,
        kind,
    }
}

fn frozen_trace() -> Vec<OutcomeEvent> {
    vec![
        event(
            "forecast-001",
            "claim-forecast-001",
            IssuerRole::ForecastService,
            10,
            10,
            OutcomeEventKind::ForecastIssued {
                model_digest: "forecast-model-sha256-v1".to_owned(),
                data_cutoff_at: 9,
                predicted_delivery_at: 100,
                delivery_interval: Some(TimeInterval {
                    earliest_at: 90,
                    latest_at: 120,
                }),
            },
        ),
        event(
            "commitment-001",
            "claim-commitment-001",
            IssuerRole::Buyer,
            20,
            20,
            OutcomeEventKind::CommitmentAuthorized {
                quantity: 5,
                due_at: 110,
                authorized_landed_cost_minor: 500,
                currency: "USD".to_owned(),
                authorization_receipt_id: "auth-receipt-001".to_owned(),
            },
        ),
        event(
            "shipment-001",
            "claim-shipment-001",
            IssuerRole::Carrier,
            80,
            81,
            OutcomeEventKind::ShipmentReported {
                quantity: 5,
                shipment_evidence_ref: "tracking-001".to_owned(),
            },
        ),
        event(
            "receipt-001",
            "claim-receipt-001",
            IssuerRole::Buyer,
            105,
            106,
            OutcomeEventKind::ReceiptObserved {
                quantity: 5,
                receipt_evidence_ref: "receiving-record-001".to_owned(),
            },
        ),
        event(
            "acceptance-001",
            "claim-acceptance-001",
            IssuerRole::IndependentVerifier,
            106,
            107,
            OutcomeEventKind::AcceptanceDecided {
                accepted_quantity: 5,
                rejected_quantity: 0,
                decision_evidence_ref: "inspection-001".to_owned(),
            },
        ),
        event(
            "finalization-001",
            "claim-finalization-001",
            IssuerRole::IndependentVerifier,
            108,
            108,
            OutcomeEventKind::OutcomeFinalized {
                net_accepted_quantity: 5,
                rejected_quantity: 0,
                realized_landed_cost_minor: 525,
                currency: "USD".to_owned(),
                accepted_at: Some(106),
                final_outcome_receipt_id: "final-outcome-001".to_owned(),
            },
        ),
    ]
}

fn replay(trace: &[OutcomeEvent], reverse: bool) -> OutcomeLedger {
    let mut ledger = OutcomeLedger::new();
    if reverse {
        for item in trace.iter().rev() {
            ledger.append(item.clone()).expect("golden event should append");
        }
    } else {
        for item in trace {
            ledger.append(item.clone()).expect("golden event should append");
        }
    }
    ledger
}

#[test]
fn frozen_happy_path_matches_independent_golden_metrics() {
    let trace = frozen_trace();
    let ledger = replay(&trace, false);
    let report = ledger
        .report_case("golden-case-001", "golden-commitment-001")
        .expect("golden event trace is internally consistent");

    assert_eq!(ledger.event_count(), 6);
    assert_eq!(report.state, OutcomeState::Finalized);
    assert_eq!(report.trusted_event_count, 6);
    assert_eq!(report.excluded_or_unverified_event_count, 0);
    assert_eq!(report.active_event_count, 6);
    assert_eq!(report.superseded_event_count, 0);
    assert_eq!(report.committed_quantity, Some(5));
    assert_eq!(report.shipped_quantity, 5);
    assert_eq!(report.received_quantity, 5);
    assert_eq!(report.gross_accepted_quantity, 5);
    assert_eq!(report.rejected_quantity, 0);
    assert_eq!(report.returned_quantity, 0);
    assert_eq!(report.net_accepted_quantity, 5);
    assert_eq!(report.on_time_in_full, Some(true));
    assert_eq!(report.forecast_delivery_error_secs, Some(6));
    assert_eq!(report.forecast_interval_covered, Some(true));
    assert_eq!(report.authorized_landed_cost_minor, Some(500));
    assert_eq!(report.realized_landed_cost_minor, Some(525));
    assert_eq!(report.landed_cost_variance_minor, Some(25));
    assert_eq!(report.final_accepted_at, Some(106));
}

#[test]
fn replay_is_independent_of_message_arrival_order_against_golden_result() {
    let trace = frozen_trace();
    let forward = replay(&trace, false)
        .report_case("golden-case-001", "golden-commitment-001")
        .expect("forward replay");
    let reverse = replay(&trace, true)
        .report_case("golden-case-001", "golden-commitment-001")
        .expect("reverse replay");

    assert_eq!(forward, reverse);
    assert_eq!(reverse.on_time_in_full, Some(true));
    assert_eq!(reverse.forecast_delivery_error_secs, Some(6));
    assert_eq!(reverse.landed_cost_variance_minor, Some(25));
}

#[test]
fn append_only_correction_preserves_event_and_replays_replacement() {
    let mut ledger = replay(&frozen_trace(), false);
    let mut correction = event(
        "receipt-002-corrected",
        "claim-receipt-correction-002",
        IssuerRole::Buyer,
        105,
        120,
        OutcomeEventKind::ReceiptObserved {
            quantity: 5,
            receipt_evidence_ref: "corrected-receiving-record-002".to_owned(),
        },
    );
    correction.supersedes_event_id = Some("receipt-001".to_owned());

    assert_eq!(
        ledger.append(correction),
        Ok(AppendDisposition::Inserted)
    );
    assert!(ledger.get_event("receipt-001").is_some());
    assert!(ledger.get_event("receipt-002-corrected").is_some());

    let report = ledger
        .report_case("golden-case-001", "golden-commitment-001")
        .expect("replacement receipt still reconciles");
    assert_eq!(ledger.event_count(), 7);
    assert_eq!(report.superseded_event_count, 1);
    assert_eq!(report.active_event_count, 6);
    assert_eq!(report.received_quantity, 5);
    assert_eq!(report.on_time_in_full, Some(true));
}

#[test]
fn synthetic_outcome_is_visible_but_never_enters_trusted_metrics() {
    let mut ledger = OutcomeLedger::new();
    let mut simulated = event(
        "synthetic-acceptance-001",
        "claim-synthetic-acceptance-001",
        IssuerRole::IndependentVerifier,
        30,
        30,
        OutcomeEventKind::AcceptanceDecided {
            accepted_quantity: 1,
            rejected_quantity: 0,
            decision_evidence_ref: "simulation-only-evidence".to_owned(),
        },
    );
    simulated.origin = EvidenceOrigin::SyntheticSimulation;

    assert_eq!(
        ledger.append(simulated),
        Ok(AppendDisposition::Inserted)
    );
    let report = ledger
        .report_case("golden-case-001", "golden-commitment-001")
        .expect("synthetic trace should remain reportable");
    assert_eq!(report.state, OutcomeState::NoTrustedEvidence);
    assert_eq!(report.trusted_event_count, 0);
    assert_eq!(report.excluded_or_unverified_event_count, 1);
    assert_eq!(report.net_accepted_quantity, 0);
    assert_eq!(report.on_time_in_full, None);
}
