//! Synthetic treatment-agnostic regenerative soil candidate frontier.
//!
//! Hard eligibility is assessed before a conservative interval-aware Pareto
//! comparison. No weighted objective and no treatment recommendation is emitted.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_agronomic_evidence";
const BOUNDARY: &str = "Synthetic candidate comparison only. A Pareto frontier is not a treatment recommendation or authorization. Thresholds and candidate effects are manufactured; real eligibility requires qualified local standards, material quality, soil/crop/site evidence and independent prospective validation.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    frontier_contract: FrontierContract,
    outcome_definitions: Vec<OutcomeDefinition>,
    candidates: Vec<Candidate>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FrontierContract {
    threshold_authority: String,
    threshold_status: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct OutcomeDefinition {
    outcome_id: String,
    unit: String,
    direction: String,
    target_min: Option<f64>,
    target_max: Option<f64>,
    hard_minimum: Option<f64>,
    hard_maximum: Option<f64>,
    hard_guardrail_status: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Candidate {
    candidate_id: String,
    candidate_class: String,
    domain_status: String,
    critical_evidence_status: String,
    product_quality_status: String,
    sourcing_status: String,
    outcomes: Vec<CandidateOutcome>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CandidateOutcome {
    outcome_id: String,
    lower_bound: f64,
    upper_bound: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    candidate_assessments: Vec<CandidateAssessment>,
    pareto_frontier_candidate_ids: Vec<String>,
    no_weighted_score_emitted: bool,
    threshold_status: &'static str,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct CandidateAssessment {
    candidate_id: String,
    eligibility_status: &'static str,
    reasons: Vec<String>,
    dominated_by: Vec<String>,
    on_frontier: bool,
}

#[derive(Debug, Serialize)]
struct Controls {
    hard_gate_before_pareto_pass: bool,
    unknown_never_pass_pass: bool,
    interval_overlap_guardrail_abstention_pass: bool,
    full_outcome_coverage_pass: bool,
    no_weighted_aggregate_pass: bool,
    no_action_baseline_supported: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if !value.is_finite() {
        return Err(format!("{name} must be finite"));
    }
    Ok(())
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "intervention-frontier-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
        || s.frontier_contract.threshold_status != "synthetic_fixture_only"
    {
        return Err("v0.1 accepts only the declared synthetic schema and thresholds".into());
    }
    if s.scenario_id.trim().is_empty()
        || s.frontier_contract.threshold_authority.trim().is_empty()
        || s.outcome_definitions.is_empty()
        || s.candidates.is_empty()
    {
        return Err(
            "scenario, threshold authority, outcomes and candidates must be non-empty".into(),
        );
    }

    let mut outcome_ids = BTreeSet::new();
    for outcome in &s.outcome_definitions {
        if outcome.outcome_id.trim().is_empty()
            || outcome.unit.trim().is_empty()
            || !outcome_ids.insert(outcome.outcome_id.clone())
        {
            return Err("outcome IDs/units must be non-empty and outcome IDs unique".into());
        }
        if !matches!(
            outcome.direction.as_str(),
            "maximize" | "minimize" | "target_range"
        ) {
            return Err(format!("{} has unsupported direction", outcome.outcome_id));
        }
        for (name, value) in [
            ("target_min", outcome.target_min),
            ("target_max", outcome.target_max),
            ("hard_minimum", outcome.hard_minimum),
            ("hard_maximum", outcome.hard_maximum),
        ] {
            if let Some(value) = value {
                finite(name, value)?;
            }
        }
        if outcome
            .hard_minimum
            .zip(outcome.hard_maximum)
            .is_some_and(|(a, b)| a > b)
        {
            return Err(format!("{} has reversed hard limits", outcome.outcome_id));
        }
        if outcome.direction == "target_range"
            && (outcome.target_min.is_none()
                || outcome.target_max.is_none()
                || outcome.target_min.unwrap_or(0.0) > outcome.target_max.unwrap_or(0.0))
        {
            return Err(format!(
                "{} requires an ordered target range",
                outcome.outcome_id
            ));
        }
        if !matches!(
            outcome.hard_guardrail_status.as_str(),
            "not_set" | "synthetic_fixture_only"
        ) {
            return Err("unsupported hard guardrail status".into());
        }
        if (outcome.hard_minimum.is_some() || outcome.hard_maximum.is_some())
            != (outcome.hard_guardrail_status == "synthetic_fixture_only")
        {
            return Err(format!(
                "{} hard bounds and guardrail status are inconsistent",
                outcome.outcome_id
            ));
        }
    }

    let mut candidate_ids = BTreeSet::new();
    for candidate in &s.candidates {
        if candidate.candidate_id.trim().is_empty()
            || !candidate_ids.insert(candidate.candidate_id.clone())
        {
            return Err("candidate IDs must be non-empty and unique".into());
        }
        if !matches!(
            candidate.candidate_class.as_str(),
            "no_action"
                | "management"
                | "compost"
                | "biochar"
                | "mineral_amendment"
                | "water_management"
                | "other"
        ) {
            return Err(format!(
                "{} has unsupported candidate class",
                candidate.candidate_id
            ));
        }
        if !matches!(
            candidate.domain_status.as_str(),
            "in_domain" | "out_of_domain" | "unassessed"
        ) || !matches!(
            candidate.critical_evidence_status.as_str(),
            "complete" | "incomplete" | "adverse"
        ) || !matches!(
            candidate.product_quality_status.as_str(),
            "not_applicable" | "profile_pass" | "profile_fail" | "unknown"
        ) || !matches!(
            candidate.sourcing_status.as_str(),
            "approved" | "not_applicable" | "unknown" | "prohibited"
        ) {
            return Err(format!(
                "{} has an unsupported evidence state",
                candidate.candidate_id
            ));
        }

        let product_required = matches!(
            candidate.candidate_class.as_str(),
            "compost" | "biochar" | "mineral_amendment"
        );
        if product_required && candidate.product_quality_status == "not_applicable" {
            return Err(format!(
                "{} requires a product-quality evidence state",
                candidate.candidate_id
            ));
        }
        if !product_required && candidate.product_quality_status != "not_applicable" {
            return Err(format!(
                "{} must not claim product quality for a non-product candidate",
                candidate.candidate_id
            ));
        }
        let sourcing_required = matches!(
            candidate.candidate_class.as_str(),
            "compost" | "biochar" | "mineral_amendment"
        );
        if sourcing_required && candidate.sourcing_status == "not_applicable" {
            return Err(format!(
                "{} requires a sourcing evidence state",
                candidate.candidate_id
            ));
        }
        if !sourcing_required && candidate.sourcing_status != "not_applicable" {
            return Err(format!(
                "{} must use not_applicable sourcing semantics",
                candidate.candidate_id
            ));
        }

        let mut seen = BTreeSet::new();
        for estimate in &candidate.outcomes {
            if !outcome_ids.contains(&estimate.outcome_id)
                || !seen.insert(estimate.outcome_id.clone())
            {
                return Err(format!(
                    "{} has an unknown or duplicate outcome estimate",
                    candidate.candidate_id
                ));
            }
            finite("outcome lower bound", estimate.lower_bound)?;
            finite("outcome upper bound", estimate.upper_bound)?;
            if estimate.lower_bound > estimate.upper_bound {
                return Err(format!(
                    "{} has reversed interval for {}",
                    candidate.candidate_id, estimate.outcome_id
                ));
            }
        }
    }
    Ok(())
}

fn candidate_outcomes_complete(candidate: &Candidate, s: &Scenario) -> bool {
    candidate.outcomes.len() == s.outcome_definitions.len()
        && s.outcome_definitions.iter().all(|definition| {
            candidate
                .outcomes
                .iter()
                .any(|estimate| estimate.outcome_id == definition.outcome_id)
        })
}

fn assess_candidate(candidate: &Candidate, s: &Scenario) -> CandidateAssessment {
    let mut blocked = Vec::new();
    let mut unresolved = Vec::new();

    match candidate.domain_status.as_str() {
        "out_of_domain" => {
            blocked.push("candidate is outside the assessed applicability domain".into())
        }
        "unassessed" => unresolved.push("candidate applicability domain is unassessed".into()),
        _ => {}
    }
    match candidate.critical_evidence_status.as_str() {
        "adverse" => blocked.push("critical evidence is adverse".into()),
        "incomplete" => unresolved.push("critical evidence is incomplete".into()),
        _ => {}
    }
    match candidate.product_quality_status.as_str() {
        "profile_fail" => blocked.push("product fails its declared quality profile".into()),
        "unknown" => unresolved.push("product quality is unknown".into()),
        _ => {}
    }
    match candidate.sourcing_status.as_str() {
        "prohibited" => {
            blocked.push("sourcing is prohibited by the declared evidence profile".into())
        }
        "unknown" => unresolved.push("sourcing or ecological-retention evidence is unknown".into()),
        _ => {}
    }

    if !candidate_outcomes_complete(candidate, s) {
        unresolved.push("one or more required outcome intervals are missing".into());
    }

    for definition in &s.outcome_definitions {
        let Some(value) = candidate
            .outcomes
            .iter()
            .find(|value| value.outcome_id == definition.outcome_id)
        else {
            continue;
        };
        if let Some(minimum) = definition.hard_minimum {
            if value.upper_bound < minimum {
                blocked.push(format!(
                    "{} is below its hard minimum even at the upper bound",
                    definition.outcome_id
                ));
            } else if value.lower_bound < minimum {
                unresolved.push(format!(
                    "{} interval overlaps its hard minimum",
                    definition.outcome_id
                ));
            }
        }
        if let Some(maximum) = definition.hard_maximum {
            if value.lower_bound > maximum {
                blocked.push(format!(
                    "{} exceeds its hard maximum even at the lower bound",
                    definition.outcome_id
                ));
            } else if value.upper_bound > maximum {
                unresolved.push(format!(
                    "{} interval overlaps its hard maximum",
                    definition.outcome_id
                ));
            }
        }
    }

    let status = if !blocked.is_empty() {
        "blocked"
    } else if !unresolved.is_empty() {
        "unresolved"
    } else {
        "eligible"
    };
    blocked.extend(unresolved);
    CandidateAssessment {
        candidate_id: candidate.candidate_id.clone(),
        eligibility_status: status,
        reasons: blocked,
        dominated_by: Vec::new(),
        on_frontier: false,
    }
}

fn interval_distance(value: f64, minimum: f64, maximum: f64) -> f64 {
    if value < minimum {
        minimum - value
    } else if value > maximum {
        value - maximum
    } else {
        0.0
    }
}

fn target_worst_distance(interval: &CandidateOutcome, definition: &OutcomeDefinition) -> f64 {
    let min = definition.target_min.unwrap_or(0.0);
    let max = definition.target_max.unwrap_or(0.0);
    interval_distance(interval.lower_bound, min, max).max(interval_distance(
        interval.upper_bound,
        min,
        max,
    ))
}

fn target_best_distance(interval: &CandidateOutcome, definition: &OutcomeDefinition) -> f64 {
    let min = definition.target_min.unwrap_or(0.0);
    let max = definition.target_max.unwrap_or(0.0);
    if interval.upper_bound >= min && interval.lower_bound <= max {
        0.0
    } else {
        interval_distance(interval.lower_bound, min, max).min(interval_distance(
            interval.upper_bound,
            min,
            max,
        ))
    }
}

/// Returns (robustly no worse, strictly better) for one objective.
fn compare_interval(
    a: &CandidateOutcome,
    b: &CandidateOutcome,
    definition: &OutcomeDefinition,
) -> (bool, bool) {
    match definition.direction.as_str() {
        "maximize" => (
            a.lower_bound >= b.upper_bound,
            a.lower_bound > b.upper_bound,
        ),
        "minimize" => (
            a.upper_bound <= b.lower_bound,
            a.upper_bound < b.lower_bound,
        ),
        "target_range" => {
            let worst_a = target_worst_distance(a, definition);
            let best_b = target_best_distance(b, definition);
            (worst_a <= best_b, worst_a < best_b)
        }
        _ => (false, false),
    }
}

fn robustly_dominates(a: &Candidate, b: &Candidate, s: &Scenario) -> bool {
    let mut strict = false;
    for definition in &s.outcome_definitions {
        let Some(a_value) = a
            .outcomes
            .iter()
            .find(|value| value.outcome_id == definition.outcome_id)
        else {
            return false;
        };
        let Some(b_value) = b
            .outcomes
            .iter()
            .find(|value| value.outcome_id == definition.outcome_id)
        else {
            return false;
        };
        let (no_worse, strictly_better) = compare_interval(a_value, b_value, definition);
        if !no_worse {
            return false;
        }
        strict |= strictly_better;
    }
    strict
}

fn build_report(s: &Scenario, digest: String) -> Report {
    let mut assessments: Vec<CandidateAssessment> = s
        .candidates
        .iter()
        .map(|candidate| assess_candidate(candidate, s))
        .collect();

    let eligible: Vec<&Candidate> = s
        .candidates
        .iter()
        .filter(|candidate| {
            assessments
                .iter()
                .find(|assessment| assessment.candidate_id == candidate.candidate_id)
                .is_some_and(|assessment| assessment.eligibility_status == "eligible")
        })
        .collect();

    let mut frontier_ids = Vec::new();
    for assessment in &mut assessments {
        if assessment.eligibility_status != "eligible" {
            continue;
        }
        for other in &eligible {
            if other.candidate_id != assessment.candidate_id
                && robustly_dominates(
                    other,
                    &s.candidates
                        .iter()
                        .find(|x| x.candidate_id == assessment.candidate_id)
                        .unwrap(),
                    s,
                )
            {
                assessment.dominated_by.push(other.candidate_id.clone());
            }
        }
        assessment.dominated_by.sort();
        assessment.on_frontier = assessment.dominated_by.is_empty();
        if assessment.on_frontier {
            frontier_ids.push(assessment.candidate_id.clone());
        }
    }
    frontier_ids.sort();

    let has_no_action = s.candidates.iter().any(|candidate| {
        candidate.candidate_class == "no_action"
            && assessments.iter().any(|assessment| {
                assessment.candidate_id == candidate.candidate_id
                    && assessment.eligibility_status == "eligible"
            })
    });

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256: digest,
        candidate_assessments: assessments,
        pareto_frontier_candidate_ids: frontier_ids,
        no_weighted_score_emitted: true,
        threshold_status: "synthetic_fixture_only_not_local_thresholds",
        controls: Controls {
            hard_gate_before_pareto_pass: true,
            unknown_never_pass_pass: true,
            interval_overlap_guardrail_abstention_pass: true,
            full_outcome_coverage_pass: true,
            no_weighted_aggregate_pass: true,
            no_action_baseline_supported: has_no_action,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input: &Path, output: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output.exists() {
        return Err(
            io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into(),
        );
    }
    let bytes = fs::read(input)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest);
    let json = serde_json::to_vec_pretty(&report)?;
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)?;
    file.write_all(&json)?;
    file.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: intervention-frontier <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("intervention frontier failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!(
            "../fixtures/synthetic-intervention-frontier.json"
        ))
        .expect("synthetic fixture parses")
    }

    fn assessment<'a>(report: &'a Report, id: &str) -> &'a CandidateAssessment {
        report
            .candidate_assessments
            .iter()
            .find(|x| x.candidate_id == id)
            .unwrap()
    }

    #[test]
    fn synthetic_frontier_excludes_blocked_and_unresolved_candidates() {
        let s = scenario();
        validate(&s).expect("valid scenario");
        let report = build_report(&s, "0".repeat(64));
        assert!(
            !report
                .pareto_frontier_candidate_ids
                .contains(&"biochar-uncertain-chloride".into())
        );
        assert!(
            !report
                .pareto_frontier_candidate_ids
                .contains(&"compost-quality-unknown".into())
        );
        assert!(
            !report
                .pareto_frontier_candidate_ids
                .contains(&"biochar-failed-quality".into())
        );
    }

    #[test]
    fn unknown_quality_and_chloride_overlap_are_unresolved() {
        let report = build_report(&scenario(), "0".repeat(64));
        assert_eq!(
            assessment(&report, "compost-quality-unknown").eligibility_status,
            "unresolved"
        );
        assert_eq!(
            assessment(&report, "biochar-uncertain-chloride").eligibility_status,
            "unresolved"
        );
    }

    #[test]
    fn failed_product_quality_is_blocked() {
        let report = build_report(&scenario(), "0".repeat(64));
        assert_eq!(
            assessment(&report, "biochar-failed-quality").eligibility_status,
            "blocked"
        );
    }

    #[test]
    fn management_candidate_robustly_dominates_no_action_but_does_not_become_a_recommendation() {
        let report = build_report(&scenario(), "0".repeat(64));
        assert!(!assessment(&report, "no-action").on_frontier);
        assert!(
            assessment(&report, "no-action")
                .dominated_by
                .contains(&"residue-cover-management".into())
        );
        assert!(report.no_weighted_score_emitted);
    }

    #[test]
    fn trade_off_candidates_remain_on_frontier() {
        let report = build_report(&scenario(), "0".repeat(64));
        assert!(assessment(&report, "residue-cover-management").on_frontier);
        assert!(assessment(&report, "gypsum-candidate").on_frontier);
        assert!(assessment(&report, "biochar-low-chloride").on_frontier);
    }

    #[test]
    fn interval_overlap_at_hard_bound_is_not_eligible() {
        let mut s = scenario();
        let candidate = s
            .candidates
            .iter_mut()
            .find(|x| x.candidate_id == "biochar-low-chloride")
            .unwrap();
        let cl = candidate
            .outcomes
            .iter_mut()
            .find(|x| x.outcome_id == "chloride_index")
            .unwrap();
        cl.upper_bound = 6.2;
        let report = build_report(&s, "0".repeat(64));
        assert_eq!(
            assessment(&report, "biochar-low-chloride").eligibility_status,
            "unresolved"
        );
    }

    #[test]
    fn unknown_domain_is_not_eligible() {
        let mut s = scenario();
        s.candidates[1].domain_status = "unassessed".into();
        let report = build_report(&s, "0".repeat(64));
        assert_eq!(
            assessment(&report, "residue-cover-management").eligibility_status,
            "unresolved"
        );
    }

    #[test]
    fn missing_outcome_prevents_pareto_membership() {
        let mut s = scenario();
        s.candidates[1].outcomes.pop();
        let report = build_report(&s, "0".repeat(64));
        assert_eq!(assessment(&report, "residue-cover-management").eligibility_status, "unresolved");
        assert!(!assessment(&report, "residue-cover-management").on_frontier);
    }

    #[test]
    fn duplicate_candidate_and_outcome_ids_are_rejected() {
        let mut s = scenario();
        s.candidates[1].candidate_id = "no-action".into();
        assert!(validate(&s).is_err());

        let mut t = scenario();
        t.candidates[0].outcomes[1].outcome_id = "crop_yield".into();
        assert!(validate(&t).is_err());
    }
}
