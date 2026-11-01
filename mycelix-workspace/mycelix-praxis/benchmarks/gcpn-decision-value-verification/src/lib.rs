//! Offline decision-value evaluation for probabilistic forecast outputs.
//!
//! This engine evaluates forecasts under a frozen synthetic cost/loss
//! decision profile. It does not set real warning thresholds, recommend real
//! actions, issue alerts or infer human risk preferences.

use std::collections::BTreeSet;

#[derive(Debug, Clone, PartialEq)]
pub struct CostLossProfile {
    pub profile_id: String,
    /// Cost of taking the synthetic action, incurred whether or not the event occurs.
    pub action_cost: f64,
    /// Avoidable loss if the event occurs and no action is taken.
    pub avoidable_event_loss: f64,
    /// Residual loss if action is taken and the event still occurs.
    pub residual_event_loss_under_action: f64,
    /// Minimum interval between decision cutoff and event valid time.
    pub minimum_action_lead_seconds: u64,
}

impl CostLossProfile {
    pub fn validate(&self) -> Result<(), String> {
        if self.profile_id.trim().is_empty() {
            return Err("decision profile ID must not be empty".into());
        }
        for (name, value) in [
            ("action_cost", self.action_cost),
            ("avoidable_event_loss", self.avoidable_event_loss),
            (
                "residual_event_loss_under_action",
                self.residual_event_loss_under_action,
            ),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(format!("{name} must be finite and nonnegative"));
            }
        }
        if self.avoidable_event_loss <= self.residual_event_loss_under_action {
            return Err("avoidable event loss must exceed residual loss under action".into());
        }
        if self.action_cost > self.avoidable_event_loss - self.residual_event_loss_under_action {
            return Err("action cost exceeds the maximum event-loss reduction".into());
        }
        Ok(())
    }

    /// Analytic switching threshold for the declared two-action loss matrix:
    /// act when p * (avoidable - residual) >= action_cost.
    pub fn decision_threshold(&self) -> Result<f64, String> {
        self.validate()?;
        Ok(self.action_cost / (self.avoidable_event_loss - self.residual_event_loss_under_action))
    }

    pub fn loss(&self, action: bool, event: bool) -> f64 {
        match (action, event) {
            (false, false) => 0.0,
            (false, true) => self.avoidable_event_loss,
            (true, false) => self.action_cost,
            (true, true) => self.action_cost + self.residual_event_loss_under_action,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct DecisionCase {
    pub case_id: String,
    pub independent_system_id: String,
    pub verification_source: String,
    pub verification_version: String,
    pub issue_time: u64,
    pub valid_time: u64,
    pub decision_cutoff: u64,
    pub candidate_probability: f64,
    pub candidate_issued_at: u64,
    pub reference_probability: f64,
    pub reference_issued_at: u64,
    pub climatology_probability: f64,
    pub event_observed: bool,
    pub action_feasible: bool,
}

impl DecisionCase {
    pub fn validate(&self) -> Result<(), String> {
        if self.case_id.trim().is_empty() || self.independent_system_id.trim().is_empty() {
            return Err("case and independent-system identifiers are required".into());
        }
        if self.verification_source.trim().is_empty() || self.verification_version.trim().is_empty()
        {
            return Err("verification source and version are required".into());
        }
        if self.issue_time >= self.valid_time {
            return Err(format!(
                "case {} must have issue time before valid time",
                self.case_id
            ));
        }
        if self.decision_cutoff < self.issue_time || self.decision_cutoff > self.valid_time {
            return Err(format!(
                "case {} has decision cutoff outside issue/valid interval",
                self.case_id
            ));
        }
        for (name, probability) in [
            ("candidate_probability", self.candidate_probability),
            ("reference_probability", self.reference_probability),
            ("climatology_probability", self.climatology_probability),
        ] {
            if !probability.is_finite() || !(0.0..=1.0).contains(&probability) {
                return Err(format!("case {} has invalid {name}", self.case_id));
            }
        }
        Ok(())
    }

    fn action_lead_is_sufficient(&self, profile: &CostLossProfile) -> bool {
        self.valid_time - self.decision_cutoff >= profile.minimum_action_lead_seconds
    }

    fn forecast_is_available_by_cutoff(&self, issued_at: u64) -> bool {
        issued_at <= self.decision_cutoff && issued_at < self.valid_time
    }

    fn policy_usable(&self, profile: &CostLossProfile, issued_at: u64) -> bool {
        self.action_feasible
            && self.action_lead_is_sufficient(profile)
            && self.forecast_is_available_by_cutoff(issued_at)
    }

    fn event_indicator(&self) -> f64 {
        if self.event_observed { 1.0 } else { 0.0 }
    }

    fn brier(&self, probability: f64) -> f64 {
        (probability - self.event_indicator()).powi(2)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolicyKind {
    Candidate,
    Reference,
    Climatology,
    NeverAct,
    AlwaysActWhenFeasible,
    PerfectInformation,
}

impl PolicyKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Candidate => "candidate_forecast_policy",
            Self::Reference => "reference_forecast_policy",
            Self::Climatology => "climatology_policy",
            Self::NeverAct => "never_act",
            Self::AlwaysActWhenFeasible => "always_act_when_feasible",
            Self::PerfectInformation => "perfect_information_infeasible_oracle",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PolicyOutcome {
    pub policy: PolicyKind,
    pub cases: u32,
    pub usable_decision_cases: u32,
    pub late_forecasts: u32,
    pub insufficient_action_lead_cases: u32,
    pub infeasible_action_cases: u32,
    pub actions_taken: u32,
    pub false_actions: u32,
    pub missed_events: u32,
    pub total_loss: f64,
    pub mean_loss_per_case: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct DecisionValueReport {
    pub profile_id: String,
    pub verification_source: String,
    pub verification_version: String,
    pub action_cost: f64,
    pub avoidable_event_loss: f64,
    pub residual_event_loss_under_action: f64,
    pub minimum_action_lead_seconds: u64,
    pub decision_threshold: f64,
    pub forecast_score_paired_cases: u32,
    pub candidate_brier_score: Option<f64>,
    pub reference_brier_score: Option<f64>,
    pub candidate: PolicyOutcome,
    pub reference: PolicyOutcome,
    pub climatology: PolicyOutcome,
    pub never_act: PolicyOutcome,
    pub always_act: PolicyOutcome,
    pub perfect_information: PolicyOutcome,
    /// Positive means the named baseline incurred more loss than candidate.
    pub net_benefit_vs_reference: f64,
    pub net_benefit_vs_climatology: f64,
    pub net_benefit_vs_never_act: f64,
    pub net_benefit_vs_always_act: f64,
    /// Candidate loss above the same-profile perfect-information lower bound.
    pub decision_regret_vs_perfect_information: f64,
}

#[derive(Debug, Clone, Copy)]
enum PolicyForecast {
    Candidate,
    Reference,
    Climatology,
    Never,
    Always,
    Oracle,
}

fn decide(
    case: &DecisionCase,
    profile: &CostLossProfile,
    policy: PolicyForecast,
    threshold: f64,
) -> (bool, bool, bool) {
    let issue_time = match policy {
        PolicyForecast::Candidate => Some(case.candidate_issued_at),
        PolicyForecast::Reference => Some(case.reference_issued_at),
        PolicyForecast::Climatology
        | PolicyForecast::Never
        | PolicyForecast::Always
        | PolicyForecast::Oracle => None,
    };

    let lead_sufficient = case.action_lead_is_sufficient(profile);
    let usable = match issue_time {
        Some(issued_at) => case.policy_usable(profile, issued_at),
        None => case.action_feasible && lead_sufficient,
    };
    let late = issue_time.is_some_and(|issued_at| !case.forecast_is_available_by_cutoff(issued_at));
    let insufficient_lead = !lead_sufficient;
    let infeasible = !case.action_feasible;

    let action = match policy {
        PolicyForecast::Candidate => usable && case.candidate_probability >= threshold,
        PolicyForecast::Reference => usable && case.reference_probability >= threshold,
        PolicyForecast::Climatology => usable && case.climatology_probability >= threshold,
        PolicyForecast::Never => false,
        PolicyForecast::Always => usable,
        PolicyForecast::Oracle => {
            if !usable {
                false
            } else {
                profile.loss(true, case.event_observed) < profile.loss(false, case.event_observed)
            }
        }
    };
    (action, usable, late)
}

fn outcome_for(
    cases: &[DecisionCase],
    profile: &CostLossProfile,
    policy: PolicyKind,
    selector: PolicyForecast,
    threshold: f64,
) -> PolicyOutcome {
    let mut result = PolicyOutcome {
        policy,
        cases: cases.len() as u32,
        usable_decision_cases: 0,
        late_forecasts: 0,
        insufficient_action_lead_cases: 0,
        infeasible_action_cases: 0,
        actions_taken: 0,
        false_actions: 0,
        missed_events: 0,
        total_loss: 0.0,
        mean_loss_per_case: 0.0,
    };

    for case in cases {
        let (action, usable, late) = decide(case, profile, selector, threshold);
        if usable {
            result.usable_decision_cases += 1;
        }
        if late {
            result.late_forecasts += 1;
        }
        if !case.action_lead_is_sufficient(profile) {
            result.insufficient_action_lead_cases += 1;
        }
        if !case.action_feasible {
            result.infeasible_action_cases += 1;
        }
        if action {
            result.actions_taken += 1;
            if !case.event_observed {
                result.false_actions += 1;
            }
        } else if case.event_observed {
            result.missed_events += 1;
        }
        result.total_loss += profile.loss(action, case.event_observed);
    }
    if !cases.is_empty() {
        result.mean_loss_per_case = result.total_loss / cases.len() as f64;
    }
    result
}

pub fn evaluate_decision_value(
    cases: &[DecisionCase],
    profile: &CostLossProfile,
) -> Result<DecisionValueReport, String> {
    evaluate_decision_value_at_threshold(cases, profile, profile.decision_threshold()?)
}

/// Evaluate the candidate policy at one explicitly supplied threshold.
/// The threshold is an analytical or predeclared sensitivity value; callers
/// must not tune it on the confirmatory sample.
pub fn evaluate_decision_value_at_threshold(
    cases: &[DecisionCase],
    profile: &CostLossProfile,
    candidate_threshold: f64,
) -> Result<DecisionValueReport, String> {
    profile.validate()?;
    if cases.is_empty() {
        return Err("decision-value case set must not be empty".into());
    }
    if !candidate_threshold.is_finite() || !(0.0..=1.0).contains(&candidate_threshold) {
        return Err("candidate decision threshold must be finite and in [0, 1]".into());
    }
    let mut ids = BTreeSet::new();
    let verification_source = cases[0].verification_source.clone();
    let verification_version = cases[0].verification_version.clone();
    for case in cases {
        case.validate()?;
        if !ids.insert(case.case_id.as_str()) {
            return Err(format!("duplicate decision case ID: {}", case.case_id));
        }
        if case.verification_source != verification_source
            || case.verification_version != verification_version
        {
            return Err("decision panel mixes verification sources or versions".into());
        }
    }
    let threshold = candidate_threshold;

    let paired: Vec<&DecisionCase> = cases
        .iter()
        .filter(|case| {
            case.candidate_issued_at < case.valid_time && case.reference_issued_at < case.valid_time
        })
        .collect();
    let candidate_brier_score = if paired.is_empty() {
        None
    } else {
        Some(
            paired
                .iter()
                .map(|case| case.brier(case.candidate_probability))
                .sum::<f64>()
                / paired.len() as f64,
        )
    };
    let reference_brier_score = if paired.is_empty() {
        None
    } else {
        Some(
            paired
                .iter()
                .map(|case| case.brier(case.reference_probability))
                .sum::<f64>()
                / paired.len() as f64,
        )
    };

    let candidate = outcome_for(
        cases,
        profile,
        PolicyKind::Candidate,
        PolicyForecast::Candidate,
        threshold,
    );
    let reference = outcome_for(
        cases,
        profile,
        PolicyKind::Reference,
        PolicyForecast::Reference,
        threshold,
    );
    let climatology = outcome_for(
        cases,
        profile,
        PolicyKind::Climatology,
        PolicyForecast::Climatology,
        threshold,
    );
    let never_act = outcome_for(
        cases,
        profile,
        PolicyKind::NeverAct,
        PolicyForecast::Never,
        threshold,
    );
    let always_act = outcome_for(
        cases,
        profile,
        PolicyKind::AlwaysActWhenFeasible,
        PolicyForecast::Always,
        threshold,
    );
    let perfect_information = outcome_for(
        cases,
        profile,
        PolicyKind::PerfectInformation,
        PolicyForecast::Oracle,
        threshold,
    );

    Ok(DecisionValueReport {
        profile_id: profile.profile_id.clone(),
        verification_source,
        verification_version,
        action_cost: profile.action_cost,
        avoidable_event_loss: profile.avoidable_event_loss,
        residual_event_loss_under_action: profile.residual_event_loss_under_action,
        minimum_action_lead_seconds: profile.minimum_action_lead_seconds,
        decision_threshold: threshold,
        forecast_score_paired_cases: paired.len() as u32,
        candidate_brier_score,
        reference_brier_score,
        net_benefit_vs_reference: reference.total_loss - candidate.total_loss,
        net_benefit_vs_climatology: climatology.total_loss - candidate.total_loss,
        net_benefit_vs_never_act: never_act.total_loss - candidate.total_loss,
        net_benefit_vs_always_act: always_act.total_loss - candidate.total_loss,
        decision_regret_vs_perfect_information: candidate.total_loss
            - perfect_information.total_loss,
        candidate,
        reference,
        climatology,
        never_act,
        always_act,
        perfect_information,
    })
}

#[derive(Debug, Clone, PartialEq)]
pub struct ThresholdValuePoint {
    pub candidate_threshold: f64,
    pub candidate_loss: f64,
    pub reference_loss: f64,
    pub net_benefit_vs_reference: f64,
    pub false_actions: u32,
    pub missed_events: u32,
}

/// Evaluate a caller-supplied, predeclared threshold grid. The function
/// validates uniqueness/order but never searches for or returns an optimal
/// threshold. Every point remains an explicit sensitivity result.
pub fn evaluate_threshold_grid(
    cases: &[DecisionCase],
    profile: &CostLossProfile,
    thresholds: &[f64],
) -> Result<Vec<ThresholdValuePoint>, String> {
    if thresholds.is_empty() {
        return Err("threshold grid must not be empty".into());
    }
    let mut previous = -1.0;
    for threshold in thresholds {
        if !threshold.is_finite() || !(0.0..=1.0).contains(threshold) {
            return Err("threshold grid values must be finite and in [0, 1]".into());
        }
        if *threshold <= previous {
            return Err("threshold grid must be strictly increasing without duplicates".into());
        }
        previous = *threshold;
    }
    let mut points = Vec::with_capacity(thresholds.len());
    for threshold in thresholds {
        let report = evaluate_decision_value_at_threshold(cases, profile, *threshold)?;
        points.push(ThresholdValuePoint {
            candidate_threshold: *threshold,
            candidate_loss: report.candidate.total_loss,
            reference_loss: report.reference.total_loss,
            net_benefit_vs_reference: report.net_benefit_vs_reference,
            false_actions: report.candidate.false_actions,
            missed_events: report.candidate.missed_events,
        });
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> CostLossProfile {
        CostLossProfile {
            profile_id: "synthetic-cost-loss-20-100".into(),
            action_cost: 20.0,
            avoidable_event_loss: 100.0,
            residual_event_loss_under_action: 0.0,
            minimum_action_lead_seconds: 30,
        }
    }

    fn case(id: &str, candidate_p: f64, reference_p: f64, outcome: bool) -> DecisionCase {
        DecisionCase {
            case_id: id.into(),
            independent_system_id: "system-1".into(),
            verification_source: "synthetic-truth".into(),
            verification_version: "truth-v1".into(),
            issue_time: 0,
            valid_time: 100,
            decision_cutoff: 50,
            candidate_probability: candidate_p,
            candidate_issued_at: 40,
            reference_probability: reference_p,
            reference_issued_at: 40,
            climatology_probability: 0.5,
            event_observed: outcome,
            action_feasible: true,
        }
    }

    fn ranking_reversal_fixture() -> Vec<DecisionCase> {
        vec![
            case("event-1", 0.90, 0.21, true),
            case("event-2", 0.19, 0.21, true),
            case("nonevent-1", 0.19, 0.21, false),
            case("nonevent-2", 0.19, 0.21, false),
        ]
    }

    #[test]
    fn analytic_threshold_is_action_cost_over_avoidable_loss() {
        let p = profile();
        assert_eq!(p.decision_threshold().unwrap(), 0.2);
    }

    #[test]
    fn proper_score_and_decision_value_can_rank_models_differently() {
        let report = evaluate_decision_value(&ranking_reversal_fixture(), &profile()).unwrap();
        assert!((report.candidate_brier_score.unwrap() - 0.184575).abs() < 1e-12);
        assert!((report.reference_brier_score.unwrap() - 0.3341).abs() < 1e-12);
        assert!(report.candidate_brier_score.unwrap() < report.reference_brier_score.unwrap());
        assert!(report.candidate.total_loss > report.reference.total_loss);
        assert!(report.net_benefit_vs_reference < 0.0);
        assert_eq!(report.action_cost, 20.0);
        assert_eq!(report.avoidable_event_loss, 100.0);
        assert_eq!(report.residual_event_loss_under_action, 0.0);
        assert_eq!(report.minimum_action_lead_seconds, 30);
        assert_eq!(report.candidate.total_loss, 120.0);
        assert_eq!(report.reference.total_loss, 80.0);
        assert_eq!(report.candidate.missed_events, 1);
        assert_eq!(report.reference.false_actions, 2);
    }

    #[test]
    fn perfect_information_is_an_infeasible_lower_bound_not_a_candidate() {
        let report = evaluate_decision_value(&ranking_reversal_fixture(), &profile()).unwrap();
        assert_eq!(report.perfect_information.total_loss, 40.0);
        assert_eq!(report.decision_regret_vs_perfect_information, 80.0);
        assert_eq!(
            report.perfect_information.policy,
            PolicyKind::PerfectInformation
        );
    }

    #[test]
    fn no_action_and_always_action_baselines_are_explicit() {
        let report = evaluate_decision_value(&ranking_reversal_fixture(), &profile()).unwrap();
        assert_eq!(report.never_act.total_loss, 200.0);
        assert_eq!(report.always_act.total_loss, 80.0);
    }

    #[test]
    fn late_forecast_is_unusable_for_action_but_remains_in_score_panel_if_pre_event() {
        let mut cases = ranking_reversal_fixture();
        cases[0].candidate_issued_at = 60;
        let report = evaluate_decision_value(&cases, &profile()).unwrap();
        assert_eq!(report.candidate.late_forecasts, 1);
        assert_eq!(report.candidate.usable_decision_cases, 3);
        assert_eq!(report.forecast_score_paired_cases, 4);
        assert_eq!(report.candidate.missed_events, 2);
    }

    #[test]
    fn forecasts_after_valid_time_are_excluded_from_paired_score_denominator() {
        let mut cases = ranking_reversal_fixture();
        cases[0].candidate_issued_at = 101;
        let report = evaluate_decision_value(&cases, &profile()).unwrap();
        assert_eq!(report.forecast_score_paired_cases, 3);
        assert!(report.candidate_brier_score.is_some());
    }

    #[test]
    fn insufficient_action_lead_and_infeasible_cases_are_reported_separately() {
        let mut cases = ranking_reversal_fixture();
        cases[0].decision_cutoff = 90;
        cases[1].action_feasible = false;
        let report = evaluate_decision_value(&cases, &profile()).unwrap();
        assert_eq!(report.candidate.insufficient_action_lead_cases, 1);
        assert_eq!(report.candidate.infeasible_action_cases, 1);
    }

    #[test]
    fn undefined_cost_loss_profile_fails_closed() {
        let mut p = profile();
        p.avoidable_event_loss = 10.0;
        p.action_cost = 20.0;
        assert!(p.decision_threshold().is_err());
    }

    #[test]
    fn duplicate_case_ids_and_invalid_probabilities_are_rejected() {
        let mut cases = ranking_reversal_fixture();
        cases[1].case_id = cases[0].case_id.clone();
        assert!(evaluate_decision_value(&cases, &profile()).is_err());
        let mut cases = ranking_reversal_fixture();
        cases[0].candidate_probability = 1.5;
        assert!(evaluate_decision_value(&cases, &profile()).is_err());
    }

    #[test]
    fn threshold_grid_is_predeclared_ordered_and_reports_each_profile_point() {
        let grid = evaluate_threshold_grid(
            &ranking_reversal_fixture(),
            &profile(),
            &[0.1, 0.2, 0.5, 0.9],
        )
        .unwrap();
        assert_eq!(grid.len(), 4);
        assert_eq!(grid[1].candidate_threshold, 0.2);
        assert_eq!(grid[1].candidate_loss, 120.0);
        assert!(
            evaluate_threshold_grid(&ranking_reversal_fixture(), &profile(), &[0.2, 0.1]).is_err()
        );
        assert!(
            evaluate_threshold_grid(&ranking_reversal_fixture(), &profile(), &[0.2, 0.2]).is_err()
        );
    }

    #[test]
    fn mixed_verification_sources_are_rejected() {
        let mut cases = ranking_reversal_fixture();
        cases[1].verification_source = "other-truth".into();
        assert!(
            evaluate_decision_value(&cases, &profile())
                .unwrap_err()
                .contains("mixes verification sources")
        );
    }

    #[test]
    fn independent_system_identity_is_required() {
        let mut cases = ranking_reversal_fixture();
        cases[0].independent_system_id = " ".into();
        assert!(evaluate_decision_value(&cases, &profile()).is_err());
    }
}
