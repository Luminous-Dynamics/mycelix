use gcpn_decision_value_verification::{CostLossProfile, DecisionCase, evaluate_decision_value};

fn main() {
    let profile = CostLossProfile {
        profile_id: "synthetic-cost-loss-20-100".into(),
        action_cost: 20.0,
        avoidable_event_loss: 100.0,
        residual_event_loss_under_action: 0.0,
        minimum_action_lead_seconds: 30,
    };
    let cases = vec![
        DecisionCase {
            case_id: "event-1".into(),
            independent_system_id: "system-1".into(),
            verification_source: "synthetic-truth".into(),
            verification_version: "truth-v1".into(),
            issue_time: 0,
            valid_time: 100,
            decision_cutoff: 50,
            candidate_probability: 0.90,
            candidate_issued_at: 40,
            reference_probability: 0.21,
            reference_issued_at: 40,
            climatology_probability: 0.50,
            event_observed: true,
            action_feasible: true,
        },
        DecisionCase {
            case_id: "event-2".into(),
            independent_system_id: "system-2".into(),
            verification_source: "synthetic-truth".into(),
            verification_version: "truth-v1".into(),
            issue_time: 0,
            valid_time: 100,
            decision_cutoff: 50,
            candidate_probability: 0.19,
            candidate_issued_at: 40,
            reference_probability: 0.21,
            reference_issued_at: 40,
            climatology_probability: 0.50,
            event_observed: true,
            action_feasible: true,
        },
        DecisionCase {
            case_id: "nonevent-1".into(),
            independent_system_id: "system-3".into(),
            verification_source: "synthetic-truth".into(),
            verification_version: "truth-v1".into(),
            issue_time: 0,
            valid_time: 100,
            decision_cutoff: 50,
            candidate_probability: 0.19,
            candidate_issued_at: 40,
            reference_probability: 0.21,
            reference_issued_at: 40,
            climatology_probability: 0.50,
            event_observed: false,
            action_feasible: true,
        },
        DecisionCase {
            case_id: "nonevent-2".into(),
            independent_system_id: "system-4".into(),
            verification_source: "synthetic-truth".into(),
            verification_version: "truth-v1".into(),
            issue_time: 0,
            valid_time: 100,
            decision_cutoff: 50,
            candidate_probability: 0.19,
            candidate_issued_at: 40,
            reference_probability: 0.21,
            reference_issued_at: 40,
            climatology_probability: 0.50,
            event_observed: false,
            action_feasible: true,
        },
    ];
    let report =
        evaluate_decision_value(&cases, &profile).expect("frozen synthetic decision profile");
    println!("GCPN decision-value forecast verification v0.1");
    println!("status: synthetic_cost_loss_oracle");
    println!("profile_id: {}", report.profile_id);
    println!("action_cost: {:.2}", report.action_cost);
    println!("avoidable_event_loss: {:.2}", report.avoidable_event_loss);
    println!(
        "residual_event_loss_under_action: {:.2}",
        report.residual_event_loss_under_action
    );
    println!(
        "minimum_action_lead_seconds: {}",
        report.minimum_action_lead_seconds
    );
    println!(
        "frozen_decision_threshold: {:.6}",
        report.decision_threshold
    );
    println!(
        "candidate_brier: {:.6}",
        report.candidate_brier_score.unwrap_or(f64::NAN)
    );
    println!(
        "reference_brier: {:.6}",
        report.reference_brier_score.unwrap_or(f64::NAN)
    );
    println!("candidate_policy_loss: {:.2}", report.candidate.total_loss);
    println!("reference_policy_loss: {:.2}", report.reference.total_loss);
    println!(
        "net_benefit_vs_reference: {:.2}",
        report.net_benefit_vs_reference
    );
    println!(
        "perfect_information_infeasible_lower_bound: {:.2}",
        report.perfect_information.total_loss
    );
    println!(
        "decision_regret_vs_perfect_information: {:.2}",
        report.decision_regret_vs_perfect_information
    );
    println!("verification_source: {}", report.verification_source);
    println!("verification_version: {}", report.verification_version);
    println!(
        "non_claim: synthetic policy evaluation is not a real warning threshold or action recommendation"
    );
}
