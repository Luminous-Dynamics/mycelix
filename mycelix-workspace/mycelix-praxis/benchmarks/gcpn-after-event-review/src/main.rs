use gcpn_after_event_review::{
    EvidenceEvent, PlannedMilestone, ReviewStage, evaluate_plan, reconstruct_as_of,
};

fn main() {
    let plan = vec![
        PlannedMilestone {
            milestone_id: "forecast-issued".into(),
            stage: ReviewStage::ObservationForecast,
            planned_at: 100,
            plan_version: "plan-v1".into(),
        },
        PlannedMilestone {
            milestone_id: "warning-disseminated".into(),
            stage: ReviewStage::WarningDissemination,
            planned_at: 150,
            plan_version: "plan-v1".into(),
        },
    ];
    let evidence = vec![
        EvidenceEvent {
            evidence_id: "e-forecast".into(),
            milestone_id: "forecast-issued".into(),
            stage: ReviewStage::ObservationForecast,
            event_at: 90,
            recorded_at: 91,
            source_id: "synthetic-agency".into(),
            plan_version: "plan-v1".into(),
            verified_under_fixture_policy: true,
        },
        EvidenceEvent {
            evidence_id: "e-warning".into(),
            milestone_id: "warning-disseminated".into(),
            stage: ReviewStage::WarningDissemination,
            event_at: 180,
            recorded_at: 181,
            source_id: "synthetic-channel".into(),
            plan_version: "plan-v1".into(),
            verified_under_fixture_policy: true,
        },
    ];
    let snapshot = reconstruct_as_of(&evidence, 200);
    let findings = evaluate_plan(&plan, &snapshot);
    println!("GCPN evidence-reconstructible after-event review v0.1");
    println!("status: synthetic_review_fixture");
    println!(
        "as_of_transaction_time: {}",
        snapshot.as_of_transaction_time
    );
    println!("visible_evidence_events: {}", snapshot.events.len());
    println!("milestones: {}", findings.len());
    for finding in findings {
        println!(
            "{:?}: {:?} at {:?}",
            finding.milestone_id, finding.status, finding.observed_at
        );
    }
    println!("non_claim: no causal impact, institutional blame, or individual ranking is inferred");
}
