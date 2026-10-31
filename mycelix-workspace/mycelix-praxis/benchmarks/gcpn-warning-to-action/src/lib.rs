//! Synthetic civilian warning-to-action evidence-chain evaluator.
//! This code does not trigger actions, release funds, issue alerts or allocate assistance.

use std::collections::HashSet;

pub const FIXTURE: &str = include_str!("../fixtures/action-chain-v0.1.csv");

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decision {
    Activate,
    NoAction,
    Abstain,
}

impl Decision {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "activate" => Ok(Self::Activate),
            "no_action" => Ok(Self::NoAction),
            "abstain" => Ok(Self::Abstain),
            _ => Err(format!("invalid decision state: {value}")),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Case {
    pub case_id: String,
    pub cap_message_id: String,
    pub protocol_version: String,
    pub alert_authorized: bool,
    pub decision: Decision,
    pub decision_authorized: bool,
    pub alert_time: u64,
    pub decision_time: u64,
    pub finance_prearranged: bool,
    pub finance_authorized: bool,
    pub funds_released: bool,
    pub finance_time: u64,
    pub action_authorized: bool,
    pub action_started: bool,
    pub action_completed: bool,
    pub action_start_time: u64,
    pub action_complete_time: u64,
    pub intended_people: u32,
    pub service_records: u32,
    pub unique_people_served: u32,
    pub duplicate_service_records: u32,
    pub grievances_reported: u32,
    pub grievances_recorded: u32,
    pub grievances_resolved: u32,
    pub outcome_observed: bool,
    pub outcome_independent: bool,
    pub counterfactual_identified: bool,
    pub impact_claim_submitted: bool,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Latency {
    pub observations: u64,
    pub total_ticks: u64,
}

impl Latency {
    pub fn mean_ticks(&self) -> Option<f64> {
        if self.observations == 0 {
            None
        } else {
            Some(self.total_ticks as f64 / self.observations as f64)
        }
    }

    fn record(&mut self, start: u64, end: u64) {
        self.observations += 1;
        self.total_ticks += end - start;
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Report {
    pub case_count: u32,
    pub missing_lineage: u32,
    pub unauthorized_activations: u32,
    pub unapproved_fund_releases: u32,
    pub unauthorized_action_starts: u32,
    pub false_completions: u32,
    pub population_overclaims: u32,
    pub service_accounting_mismatches: u32,
    pub grievance_suppression: u32,
    pub grievance_resolution_overcounts: u32,
    pub impact_overclaims: u32,
    pub chronology_violations: u32,
    pub activation_count: u32,
    pub funds_released_count: u32,
    pub actions_started_count: u32,
    pub actions_completed_count: u32,
    pub intended_people_total: u64,
    pub unique_people_served_total: u64,
    pub warning_to_decision: Latency,
    pub decision_to_finance: Latency,
    pub finance_to_action: Latency,
    pub action_to_completion: Latency,
}

impl Report {
    pub fn hard_gates_pass(&self) -> bool {
        self.missing_lineage == 0
            && self.unauthorized_activations == 0
            && self.unapproved_fund_releases == 0
            && self.unauthorized_action_starts == 0
            && self.false_completions == 0
            && self.population_overclaims == 0
            && self.service_accounting_mismatches == 0
            && self.grievance_suppression == 0
            && self.grievance_resolution_overcounts == 0
            && self.impact_overclaims == 0
            && self.chronology_violations == 0
    }

    pub fn service_coverage_ratio(&self) -> f64 {
        if self.intended_people_total == 0 {
            0.0
        } else {
            self.unique_people_served_total as f64 / self.intended_people_total as f64
        }
    }
}

fn parse_bool(value: &str, field: &str) -> Result<bool, String> {
    match value {
        "true" => Ok(true),
        "false" => Ok(false),
        _ => Err(format!("{field} must be true or false, got {value}")),
    }
}

fn parse_u32(value: &str, field: &str) -> Result<u32, String> {
    value
        .parse()
        .map_err(|_| format!("{field} must be an unsigned 32-bit integer, got {value}"))
}

fn parse_u64(value: &str, field: &str) -> Result<u64, String> {
    value
        .parse()
        .map_err(|_| format!("{field} must be an unsigned 64-bit integer, got {value}"))
}

pub fn parse_fixture(input: &str) -> Result<Vec<Case>, String> {
    const HEADER: &str = "case_id,cap_message_id,protocol_version,alert_authorized,decision,decision_authorized,alert_time,decision_time,finance_prearranged,finance_authorized,funds_released,finance_time,action_authorized,action_started,action_completed,action_start_time,action_complete_time,intended_people,service_records,unique_people_served,duplicate_service_records,grievances_reported,grievances_recorded,grievances_resolved,outcome_observed,outcome_independent,counterfactual_identified,impact_claim_submitted";
    let mut lines = input.lines();
    if lines.next() != Some(HEADER) {
        return Err("warning-to-action fixture header does not match frozen v0.1 contract".into());
    }

    let mut cases = Vec::new();
    let mut ids = HashSet::new();
    for (line_idx, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() != 28 {
            return Err(format!("line {}: expected 28 fields, got {}", line_idx + 2, f.len()));
        }

        let case = Case {
            case_id: f[0].to_owned(),
            cap_message_id: f[1].to_owned(),
            protocol_version: f[2].to_owned(),
            alert_authorized: parse_bool(f[3], "alert_authorized")?,
            decision: Decision::parse(f[4])?,
            decision_authorized: parse_bool(f[5], "decision_authorized")?,
            alert_time: parse_u64(f[6], "alert_time")?,
            decision_time: parse_u64(f[7], "decision_time")?,
            finance_prearranged: parse_bool(f[8], "finance_prearranged")?,
            finance_authorized: parse_bool(f[9], "finance_authorized")?,
            funds_released: parse_bool(f[10], "funds_released")?,
            finance_time: parse_u64(f[11], "finance_time")?,
            action_authorized: parse_bool(f[12], "action_authorized")?,
            action_started: parse_bool(f[13], "action_started")?,
            action_completed: parse_bool(f[14], "action_completed")?,
            action_start_time: parse_u64(f[15], "action_start_time")?,
            action_complete_time: parse_u64(f[16], "action_complete_time")?,
            intended_people: parse_u32(f[17], "intended_people")?,
            service_records: parse_u32(f[18], "service_records")?,
            unique_people_served: parse_u32(f[19], "unique_people_served")?,
            duplicate_service_records: parse_u32(f[20], "duplicate_service_records")?,
            grievances_reported: parse_u32(f[21], "grievances_reported")?,
            grievances_recorded: parse_u32(f[22], "grievances_recorded")?,
            grievances_resolved: parse_u32(f[23], "grievances_resolved")?,
            outcome_observed: parse_bool(f[24], "outcome_observed")?,
            outcome_independent: parse_bool(f[25], "outcome_independent")?,
            counterfactual_identified: parse_bool(f[26], "counterfactual_identified")?,
            impact_claim_submitted: parse_bool(f[27], "impact_claim_submitted")?,
        };

        if case.case_id.trim().is_empty() {
            return Err(format!("line {}: case_id is required", line_idx + 2));
        }
        if !ids.insert(case.case_id.clone()) {
            return Err(format!("line {}: duplicate case_id", line_idx + 2));
        }
        if case.intended_people == 0 {
            return Err(format!("line {}: intended_people must be non-zero", line_idx + 2));
        }
        cases.push(case);
    }

    if cases.is_empty() {
        return Err("warning-to-action fixture contains no cases".into());
    }
    Ok(cases)
}

fn evaluate_case(case: &Case, report: &mut Report) {
    if case.cap_message_id.trim().is_empty() || case.protocol_version.trim().is_empty() {
        report.missing_lineage += 1;
    }

    if case.decision == Decision::Activate {
        report.activation_count += 1;
        if !case.alert_authorized || !case.decision_authorized {
            report.unauthorized_activations += 1;
        }
    }

    if case.funds_released {
        report.funds_released_count += 1;
        if case.decision != Decision::Activate
            || !case.alert_authorized
            || !case.decision_authorized
            || !case.finance_prearranged
            || !case.finance_authorized
        {
            report.unapproved_fund_releases += 1;
        }
    }

    if case.action_started {
        report.actions_started_count += 1;
        if case.decision != Decision::Activate
            || !case.decision_authorized
            || !case.action_authorized
            || !case.funds_released
        {
            report.unauthorized_action_starts += 1;
        }
    }

    if case.action_completed {
        report.actions_completed_count += 1;
        if !case.action_started {
            report.false_completions += 1;
        }
    }

    if case.unique_people_served > case.intended_people {
        report.population_overclaims += 1;
    }
    if case.service_records != case.unique_people_served.saturating_add(case.duplicate_service_records) {
        report.service_accounting_mismatches += 1;
    }

    if case.grievances_reported != case.grievances_recorded {
        report.grievance_suppression += 1;
    }
    if case.grievances_resolved > case.grievances_recorded {
        report.grievance_resolution_overcounts += 1;
    }

    if case.impact_claim_submitted
        && (!case.outcome_observed || !case.outcome_independent || !case.counterfactual_identified)
    {
        report.impact_overclaims += 1;
    }

    if case.decision_time > 0 && case.alert_time > 0 {
        if case.decision_time < case.alert_time {
            report.chronology_violations += 1;
        } else {
            report.warning_to_decision.record(case.alert_time, case.decision_time);
        }
    } else if case.decision == Decision::Activate || case.decision == Decision::NoAction || case.decision == Decision::Abstain {
        report.chronology_violations += 1;
    }

    if case.funds_released {
        if case.finance_time == 0 || case.finance_time < case.decision_time {
            report.chronology_violations += 1;
        } else {
            report.decision_to_finance.record(case.decision_time, case.finance_time);
        }
    }

    if case.action_started {
        if case.action_start_time == 0 || case.action_start_time < case.finance_time {
            report.chronology_violations += 1;
        } else {
            report.finance_to_action.record(case.finance_time, case.action_start_time);
        }
    }

    if case.action_completed && case.action_started {
        if case.action_complete_time == 0 || case.action_complete_time < case.action_start_time {
            report.chronology_violations += 1;
        } else {
            report.action_to_completion.record(case.action_start_time, case.action_complete_time);
        }
    }

    report.intended_people_total += u64::from(case.intended_people);
    report.unique_people_served_total += u64::from(case.unique_people_served);
}

pub fn evaluate(cases: &[Case]) -> Report {
    let mut report = Report {
        case_count: cases.len() as u32,
        ..Report::default()
    };
    for case in cases {
        evaluate_case(case, &mut report);
    }
    report
}

pub fn fixture_report() -> Result<Report, String> {
    Ok(evaluate(&parse_fixture(FIXTURE)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn case(id: &str) -> Case {
        parse_fixture(FIXTURE)
            .unwrap()
            .into_iter()
            .find(|case| case.case_id == id)
            .expect("case id exists in fixture")
    }

    #[test]
    fn fixture_is_unique_and_pinned() {
        fn fnv1a64(bytes: &[u8]) -> u64 {
            bytes.iter().fold(0xcbf29ce484222325u64, |mut hash, byte| {
                hash ^= u64::from(*byte);
                hash.wrapping_mul(0x100000001b3)
            })
        }
        assert_eq!(fnv1a64(FIXTURE.as_bytes()), 0);
        let cases = parse_fixture(FIXTURE).unwrap();
        assert_eq!(cases.len(), 10);
        assert_eq!(cases.iter().map(|case| case.case_id.as_str()).collect::<HashSet<_>>().len(), 10);
    }

    #[test]
    fn valid_complete_chain_has_no_integrity_violations() {
        let report = evaluate(&[case("valid-complete-chain")]);
        assert!(report.hard_gates_pass(), "{report:?}");
        assert_eq!(report.activation_count, 1);
        assert_eq!(report.funds_released_count, 1);
        assert_eq!(report.actions_started_count, 1);
        assert_eq!(report.actions_completed_count, 1);
        assert_eq!(report.unique_people_served_total, 900);
    }

    #[test]
    fn unauthorized_activation_is_rejected() {
        let report = evaluate(&[case("unauthorized-activation")]);
        assert_eq!(report.unauthorized_activations, 1);
        assert!(!report.hard_gates_pass());
    }

    #[test]
    fn unapproved_funding_release_is_rejected() {
        let report = evaluate(&[case("unapproved-fund-release")]);
        assert_eq!(report.unapproved_fund_releases, 1);
        assert!(!report.hard_gates_pass());
    }

    #[test]
    fn action_requires_authority_and_finance() {
        let report = evaluate(&[case("unauthorized-action-start")]);
        assert_eq!(report.unauthorized_action_starts, 1);
        assert!(!report.hard_gates_pass());
    }

    #[test]
    fn completion_requires_a_recorded_start() {
        let report = evaluate(&[case("false-completion")]);
        assert_eq!(report.false_completions, 1);
        assert_eq!(report.chronology_violations, 0);
        assert!(!report.hard_gates_pass());
    }

    #[test]
    fn population_overclaim_is_detected_even_with_consistent_duplicate_accounting() {
        let report = evaluate(&[case("population-overclaim")]);
        assert_eq!(report.population_overclaims, 1);
        assert_eq!(report.service_accounting_mismatches, 0);
        assert!(!report.hard_gates_pass());
    }

    #[test]
    fn suppressed_grievances_remain_visible_as_a_hard_failure() {
        let report = evaluate(&[case("grievance-suppression")]);
        assert_eq!(report.grievance_suppression, 1);
        assert!(!report.hard_gates_pass());
    }

    #[test]
    fn causal_impact_claim_requires_outcome_counterfactual_and_independent_evidence() {
        let report = evaluate(&[case("unsupported-impact-claim")]);
        assert_eq!(report.impact_overclaims, 1);
        assert!(!report.hard_gates_pass());
    }

    #[test]
    fn abstention_without_action_is_a_valid_non_activation() {
        let report = evaluate(&[case("legitimate-abstention")]);
        assert_eq!(report.activation_count, 0);
        assert_eq!(report.funds_released_count, 0);
        assert_eq!(report.actions_started_count, 0);
        assert!(report.hard_gates_pass());
    }

    #[test]
    fn out_of_order_action_timestamps_are_rejected() {
        let report = evaluate(&[case("chronology-violation")]);
        assert_eq!(report.chronology_violations, 1);
        assert!(!report.hard_gates_pass());
    }

    #[test]
    fn service_records_do_not_become_unique_people_or_proof_of_impact() {
        let report = evaluate(&[case("valid-complete-chain")]);
        assert_eq!(report.unique_people_served_total, 900);
        assert_eq!(report.intended_people_total, 1_000);
        assert!(!report.hard_gates_pass() == false);
        assert_eq!(report.impact_overclaims, 0);
    }
}
