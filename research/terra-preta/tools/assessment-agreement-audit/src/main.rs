//! Synthetic-only soil assessment disagreement and robustness audit.
//!
//! The six methods are transparent scoring archetypes, not reproductions of
//! CASH, SMAF, Soil Navigator, SAS or any commercial protocol. Scores expose
//! assumptions; they do not establish a universal soil-health truth.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic assessment-method audit only. Method-specific scores are not interchangeable or universal truth. Hard guardrails and Pareto function vectors are separate from scalar rankings. All values and limits are manufactured.";

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    indicators: Vec<Indicator>,
    weight_profiles: Vec<WeightProfile>,
    hard_guardrails: Vec<HardGuardrail>,
    alternatives: Vec<Alternative>,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct Indicator {
    indicator_id: String,
    function_group: String,
    unit: String,
    direction: String,
    reference_min: f64,
    reference_max: f64,
    threshold_value: f64,
    threshold_relation: String,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct WeightProfile {
    profile_id: String,
    weights: Vec<Weight>,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct Weight {
    indicator_id: String,
    weight: f64,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct HardGuardrail {
    guardrail_id: String,
    indicator_id: String,
    status: String,
    minimum_value: Option<f64>,
    maximum_value: Option<f64>,
    authority: String,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct Alternative {
    alternative_id: String,
    independence_group_id: String,
    measurements: Vec<Measurement>,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct Measurement {
    indicator_id: String,
    value: f64,
    standard_error: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    indicator_count: usize,
    alternative_count: usize,
    method_rankings: Vec<MethodRanking>,
    pairwise_method_agreement: Vec<PairwiseAgreement>,
    kendall_w: f64,
    mean_pairwise_spearman: f64,
    weight_profile_rankings: Vec<ProfileRanking>,
    weight_profile_pairwise_reversals: usize,
    nominal_pareto_front_ids: Vec<String>,
    robust_pareto_front_ids: Vec<String>,
    robust_dominance_relations: Vec<DominanceRelation>,
    alternative_guardrails: Vec<AlternativeGuardrails>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct MethodRanking {
    method_id: String,
    method_family: &'static str,
    interpretation_boundary: &'static str,
    ranked_alternatives: Vec<RankedAlternative>,
}

#[derive(Debug, Serialize, Clone)]
struct RankedAlternative {
    alternative_id: String,
    score: f64,
    rank: f64,
}

type ScoredAlternative = (String, f64);
type MethodScoreSet = (String, &'static str, &'static str, Vec<ScoredAlternative>);

#[derive(Debug, Serialize)]
struct PairwiseAgreement {
    method_a: String,
    method_b: String,
    spearman_rho: f64,
}

#[derive(Debug, Serialize)]
struct ProfileRanking {
    profile_id: String,
    ranked_alternatives: Vec<RankedAlternative>,
}

#[derive(Debug, Serialize)]
struct DominanceRelation {
    dominant_alternative_id: String,
    dominated_alternative_id: String,
}

#[derive(Debug, Serialize)]
struct AlternativeGuardrails {
    alternative_id: String,
    eligible: bool,
    guardrail_results: Vec<GuardrailResult>,
}

#[derive(Debug, Serialize)]
struct GuardrailResult {
    guardrail_id: String,
    status: &'static str,
}

#[derive(Debug, Serialize)]
struct Controls {
    no_method_promoted_as_universal_truth: bool,
    guardrails_separate_from_scores: bool,
    pareto_separate_from_scalar_ranks: bool,
    uncertainty_overlap_abstains: bool,
    permutation_invariant: bool,
    no_industry_reproduction_claim: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if !value.is_finite() {
        return Err(format!("{name} must be finite"));
    }
    Ok(())
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "soil-assessment-agreement-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("v0.1 only accepts the declared synthetic scenario".into());
    }
    if s.scenario_id.trim().is_empty()
        || s.indicators.len() < 2
        || s.alternatives.len() < 3
        || s.weight_profiles.len() < 2
        || s.hard_guardrails.is_empty()
    {
        return Err(
            "scenario requires indicators, alternatives, profiles and hard guardrails".into(),
        );
    }

    let mut indicator_ids = BTreeSet::new();
    for i in &s.indicators {
        if i.indicator_id.trim().is_empty()
            || i.function_group.trim().is_empty()
            || i.unit.trim().is_empty()
            || !indicator_ids.insert(i.indicator_id.clone())
        {
            return Err("indicator IDs and metadata must be non-empty and unique".into());
        }
        for (name, value) in [
            ("reference_min", i.reference_min),
            ("reference_max", i.reference_max),
            ("threshold_value", i.threshold_value),
        ] {
            finite(name, value)?;
        }
        if i.reference_max <= i.reference_min {
            return Err(format!("{} has invalid reference interval", i.indicator_id));
        }
        match (i.direction.as_str(), i.threshold_relation.as_str()) {
            ("higher_is_better", "minimum") | ("lower_is_better", "maximum") => {}
            _ => {
                return Err(format!(
                    "{} has unknown or inconsistent direction/threshold",
                    i.indicator_id
                ));
            }
        }
    }

    let mut profile_ids = BTreeSet::new();
    for p in &s.weight_profiles {
        if p.profile_id.trim().is_empty() || !profile_ids.insert(p.profile_id.clone()) {
            return Err("weight profile IDs must be non-empty and unique".into());
        }
        let mut weight_ids = BTreeSet::new();
        let mut sum = 0.0;
        for w in &p.weights {
            finite("weight", w.weight)?;
            if w.weight < 0.0
                || !indicator_ids.contains(&w.indicator_id)
                || !weight_ids.insert(w.indicator_id.clone())
            {
                return Err(format!(
                    "{} contains invalid or duplicate indicator weight",
                    p.profile_id
                ));
            }
            sum += w.weight;
        }
        if weight_ids != indicator_ids || (sum - 1.0).abs() > 1e-8 {
            return Err(format!(
                "{} must weight every indicator exactly once and sum to one",
                p.profile_id
            ));
        }
    }
    if !profile_ids.contains("declared_expert") {
        return Err(
            "a declared_expert weight profile is required for the expert-weighted archetype".into(),
        );
    }

    let mut guardrail_ids = BTreeSet::new();
    for g in &s.hard_guardrails {
        if g.guardrail_id.trim().is_empty()
            || !guardrail_ids.insert(g.guardrail_id.clone())
            || !indicator_ids.contains(&g.indicator_id)
            || g.status != "synthetic_fixture"
            || g.authority.trim().is_empty()
            || (g.minimum_value.is_none() && g.maximum_value.is_none())
        {
            return Err(
                "guardrails require unique IDs, known indicators and explicit synthetic provenance"
                    .into(),
            );
        }
        for (name, value) in [
            ("guardrail minimum", g.minimum_value),
            ("guardrail maximum", g.maximum_value),
        ] {
            if let Some(v) = value {
                finite(name, v)?;
            }
        }
        if g.minimum_value
            .zip(g.maximum_value)
            .is_some_and(|(a, b)| a > b)
        {
            return Err("guardrail minimum cannot exceed maximum".into());
        }
    }

    let mut alternative_ids = BTreeSet::new();
    for a in &s.alternatives {
        if a.alternative_id.trim().is_empty()
            || a.independence_group_id.trim().is_empty()
            || !alternative_ids.insert(a.alternative_id.clone())
        {
            return Err(
                "alternative IDs and independence groups must be non-empty and IDs unique".into(),
            );
        }
        let mut seen = BTreeSet::new();
        for m in &a.measurements {
            if !indicator_ids.contains(&m.indicator_id) || !seen.insert(m.indicator_id.clone()) {
                return Err(format!(
                    "{} has an unknown or duplicate indicator measurement",
                    a.alternative_id
                ));
            }
            finite("measurement value", m.value)?;
            finite("measurement standard_error", m.standard_error)?;
            if m.standard_error < 0.0 {
                return Err("measurement standard errors cannot be negative".into());
            }
        }
        if seen != indicator_ids {
            return Err(format!(
                "{} must contain exactly one value for every indicator",
                a.alternative_id
            ));
        }
    }
    Ok(())
}

fn oriented(value: f64, direction: &str) -> f64 {
    if direction == "higher_is_better" {
        value
    } else {
        -value
    }
}

fn normalized(value: f64, indicator: &Indicator) -> f64 {
    let raw = if indicator.direction == "higher_is_better" {
        (value - indicator.reference_min) / (indicator.reference_max - indicator.reference_min)
    } else {
        (indicator.reference_max - value) / (indicator.reference_max - indicator.reference_min)
    };
    raw.clamp(0.0, 1.0)
}

fn measurement<'a>(a: &'a Alternative, id: &str) -> &'a Measurement {
    a.measurements
        .iter()
        .find(|m| m.indicator_id == id)
        .expect("validated measurement exists")
}

fn sorted_alternatives(s: &Scenario) -> Vec<&Alternative> {
    let mut out: Vec<&Alternative> = s.alternatives.iter().collect();
    out.sort_by(|a, b| a.alternative_id.cmp(&b.alternative_id));
    out
}

fn average_ranks_desc(values: &[(String, f64)]) -> BTreeMap<String, f64> {
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| b.1.total_cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
    let mut out = BTreeMap::new();
    let mut start = 0usize;
    while start < sorted.len() {
        let mut end = start + 1;
        while end < sorted.len() && sorted[end].1 == sorted[start].1 {
            end += 1;
        }
        let average_rank = (start + 1 + end) as f64 / 2.0;
        for item in &sorted[start..end] {
            out.insert(item.0.clone(), average_rank);
        }
        start = end;
    }
    out
}

fn rank_scores(values: Vec<(String, f64)>) -> Vec<RankedAlternative> {
    let ranks = average_ranks_desc(&values);
    let mut rows: Vec<RankedAlternative> = values
        .into_iter()
        .map(|(id, score)| RankedAlternative {
            alternative_id: id.clone(),
            score,
            rank: *ranks.get(&id).expect("rank exists"),
        })
        .collect();
    rows.sort_by(|a, b| {
        a.rank
            .total_cmp(&b.rank)
            .then_with(|| a.alternative_id.cmp(&b.alternative_id))
    });
    rows
}

fn empirical_percentile_scores(s: &Scenario) -> BTreeMap<String, f64> {
    let alternatives = sorted_alternatives(s);
    let mut total = BTreeMap::new();
    for indicator in &s.indicators {
        let values: Vec<(String, f64)> = alternatives
            .iter()
            .map(|a| {
                (
                    a.alternative_id.clone(),
                    oriented(
                        measurement(a, &indicator.indicator_id).value,
                        &indicator.direction,
                    ),
                )
            })
            .collect();
        let ranks = average_ranks_desc(&values);
        let n = alternatives.len() as f64;
        for (id, rank) in ranks {
            let percentile = if n <= 1.0 {
                1.0
            } else {
                (n - rank) / (n - 1.0)
            };
            *total.entry(id).or_insert(0.0) += percentile;
        }
    }
    total
        .values_mut()
        .for_each(|x| *x /= s.indicators.len() as f64);
    total
}

fn weight_map(profile: &WeightProfile) -> BTreeMap<String, f64> {
    profile
        .weights
        .iter()
        .map(|w| (w.indicator_id.clone(), w.weight))
        .collect()
}

fn weighted_scores(s: &Scenario, profile: &WeightProfile) -> Vec<(String, f64)> {
    let weights = weight_map(profile);
    sorted_alternatives(s)
        .into_iter()
        .map(|a| {
            let score = s
                .indicators
                .iter()
                .map(|i| {
                    normalized(measurement(a, &i.indicator_id).value, i) * weights[&i.indicator_id]
                })
                .sum();
            (a.alternative_id.clone(), score)
        })
        .collect()
}

fn equal_scores(s: &Scenario) -> Vec<(String, f64)> {
    sorted_alternatives(s)
        .into_iter()
        .map(|a| {
            let score = s
                .indicators
                .iter()
                .map(|i| normalized(measurement(a, &i.indicator_id).value, i))
                .sum::<f64>()
                / s.indicators.len() as f64;
            (a.alternative_id.clone(), score)
        })
        .collect()
}

fn threshold_scores(s: &Scenario) -> Vec<(String, f64)> {
    sorted_alternatives(s)
        .into_iter()
        .map(|a| {
            let passed = s
                .indicators
                .iter()
                .filter(|i| {
                    let value = measurement(a, &i.indicator_id).value;
                    if i.threshold_relation == "minimum" {
                        value >= i.threshold_value
                    } else {
                        value <= i.threshold_value
                    }
                })
                .count();
            (
                a.alternative_id.clone(),
                passed as f64 / s.indicators.len() as f64,
            )
        })
        .collect()
}

fn limiting_scores(s: &Scenario) -> Vec<(String, f64)> {
    sorted_alternatives(s)
        .into_iter()
        .map(|a| {
            let score = s
                .indicators
                .iter()
                .map(|i| normalized(measurement(a, &i.indicator_id).value, i))
                .fold(f64::INFINITY, f64::min);
            (a.alternative_id.clone(), score)
        })
        .collect()
}

fn geometric_scores(s: &Scenario) -> Vec<(String, f64)> {
    sorted_alternatives(s)
        .into_iter()
        .map(|a| {
            let product = s
                .indicators
                .iter()
                .map(|i| normalized(measurement(a, &i.indicator_id).value, i).max(1e-12))
                .product::<f64>();
            (
                a.alternative_id.clone(),
                product.powf(1.0 / s.indicators.len() as f64),
            )
        })
        .collect()
}

fn method_scores(s: &Scenario) -> Vec<MethodScoreSet> {
    let expert = s
        .weight_profiles
        .iter()
        .find(|p| p.profile_id == "declared_expert")
        .expect("validated expert profile");
    let empirical = empirical_percentile_scores(s);
    let emp = sorted_alternatives(s)
        .into_iter()
        .map(|a| (a.alternative_id.clone(), empirical[&a.alternative_id]))
        .collect();
    vec![
        (
            "equal_linear_mean".into(),
            "scoring_function",
            "Equal-weight fixed-reference normalized average; synthetic representative only.",
            equal_scores(s),
        ),
        (
            "declared_expert_weighted_mean".into(),
            "expert_judgement",
            "Explicit declared weights; synthetic representative only.",
            weighted_scores(s, expert),
        ),
        (
            "empirical_percentile_mean".into(),
            "cohort_relative",
            "Within-cohort oriented percentile mean; changes with the comparison cohort.",
            emp,
        ),
        (
            "threshold_pass_fraction".into(),
            "threshold_rule",
            "Fraction of declared indicator thresholds passed; a failed indicator remains visible.",
            threshold_scores(s),
        ),
        (
            "limiting_function_min".into(),
            "limiting_function",
            "Minimum normalized indicator value; exposes the weakest normalized function.",
            limiting_scores(s),
        ),
        (
            "geometric_mean".into(),
            "multifunctionality",
            "Geometric mean of normalized indicators; synthetic representative only.",
            geometric_scores(s),
        ),
    ]
}

fn rank_lookup(rows: &[RankedAlternative]) -> BTreeMap<String, f64> {
    rows.iter()
        .map(|r| (r.alternative_id.clone(), r.rank))
        .collect()
}

fn pearson(x: &[f64], y: &[f64]) -> f64 {
    if x.len() != y.len() || x.len() < 2 {
        return 0.0;
    }
    let mx = x.iter().sum::<f64>() / x.len() as f64;
    let my = y.iter().sum::<f64>() / y.len() as f64;
    let cov = x
        .iter()
        .zip(y)
        .map(|(a, b)| (a - mx) * (b - my))
        .sum::<f64>();
    let vx = x.iter().map(|a| (a - mx).powi(2)).sum::<f64>();
    let vy = y.iter().map(|b| (b - my).powi(2)).sum::<f64>();
    if vx <= f64::EPSILON || vy <= f64::EPSILON {
        if x.iter().zip(y).all(|(a, b)| a == b) {
            1.0
        } else {
            0.0
        }
    } else {
        (cov / (vx * vy).sqrt()).clamp(-1.0, 1.0)
    }
}

fn spearman(a: &[RankedAlternative], b: &[RankedAlternative]) -> f64 {
    let ra = rank_lookup(a);
    let rb = rank_lookup(b);
    let ids: Vec<String> = ra.keys().cloned().collect();
    let x: Vec<f64> = ids.iter().map(|id| ra[id]).collect();
    let y: Vec<f64> = ids.iter().map(|id| rb[id]).collect();
    pearson(&x, &y)
}

fn kendall_w(rankings: &[Vec<RankedAlternative>]) -> f64 {
    if rankings.is_empty() || rankings[0].len() < 2 {
        return 1.0;
    }
    let m = rankings.len() as f64;
    let ids: Vec<String> = rankings[0]
        .iter()
        .map(|x| x.alternative_id.clone())
        .collect();
    let n = ids.len() as f64;
    let maps: Vec<BTreeMap<String, f64>> = rankings.iter().map(|r| rank_lookup(r)).collect();
    let expected = m * (n + 1.0) / 2.0;
    let s = ids
        .iter()
        .map(|id| {
            let total = maps.iter().map(|map| map[id]).sum::<f64>();
            (total - expected).powi(2)
        })
        .sum::<f64>();
    let mut tie_total = 0.0;
    for ranking in rankings {
        let mut groups: BTreeMap<i64, usize> = BTreeMap::new();
        for row in ranking {
            *groups.entry((row.rank * 2.0).round() as i64).or_insert(0) += 1;
        }
        tie_total += groups
            .values()
            .map(|t| {
                let t = *t as f64;
                t.powi(3) - t
            })
            .sum::<f64>();
    }
    let denominator = m * m * (n.powi(3) - n) - m * tie_total;
    if denominator <= f64::EPSILON {
        1.0
    } else {
        (12.0 * s / denominator).clamp(0.0, 1.0)
    }
}

fn nominally_dominates(a: &Alternative, b: &Alternative, indicators: &[Indicator]) -> bool {
    let mut strict = false;
    for i in indicators {
        let av = oriented(measurement(a, &i.indicator_id).value, &i.direction);
        let bv = oriented(measurement(b, &i.indicator_id).value, &i.direction);
        if av < bv {
            return false;
        }
        if av > bv {
            strict = true;
        }
    }
    strict
}

fn robustly_dominates(a: &Alternative, b: &Alternative, indicators: &[Indicator]) -> bool {
    let mut strict = false;
    for i in indicators {
        let am = measurement(a, &i.indicator_id);
        let bm = measurement(b, &i.indicator_id);
        let (a_low, b_high) = if i.direction == "higher_is_better" {
            (
                am.value - 1.96 * am.standard_error,
                bm.value + 1.96 * bm.standard_error,
            )
        } else {
            (
                -(am.value + 1.96 * am.standard_error),
                -(bm.value - 1.96 * bm.standard_error),
            )
        };
        const COMPARISON_EPSILON: f64 = 1e-10;
        if a_low + COMPARISON_EPSILON < b_high {
            return false;
        }
        if a_low > b_high + COMPARISON_EPSILON {
            strict = true;
        }
    }
    strict
}

fn pareto_front(s: &Scenario, robust: bool) -> Vec<String> {
    let alts = sorted_alternatives(s);
    alts.iter()
        .filter(|candidate| {
            !alts.iter().any(|other| {
                other.alternative_id != candidate.alternative_id
                    && if robust {
                        robustly_dominates(other, candidate, &s.indicators)
                    } else {
                        nominally_dominates(other, candidate, &s.indicators)
                    }
            })
        })
        .map(|a| a.alternative_id.clone())
        .collect()
}

fn guardrail_status(value: f64, se: f64, g: &HardGuardrail) -> &'static str {
    let lower = value - 1.96 * se;
    let upper = value + 1.96 * se;
    let violates_min = g.minimum_value.is_some_and(|x| upper < x);
    let uncertain_min = g.minimum_value.is_some_and(|x| lower < x);
    let violates_max = g.maximum_value.is_some_and(|x| lower > x);
    let uncertain_max = g.maximum_value.is_some_and(|x| upper > x);
    if violates_min || violates_max {
        "violated"
    } else if uncertain_min || uncertain_max {
        "uncertain"
    } else {
        "clear"
    }
}

fn build_report(s: &Scenario, digest: String) -> Report {
    let methods = method_scores(s);
    let method_rankings: Vec<MethodRanking> = methods
        .iter()
        .map(|(id, family, boundary, scores)| MethodRanking {
            method_id: id.clone(),
            method_family: family,
            interpretation_boundary: boundary,
            ranked_alternatives: rank_scores(scores.clone()),
        })
        .collect();

    let mut pairwise = Vec::new();
    let mut correlations = Vec::new();
    for i in 0..method_rankings.len() {
        for j in (i + 1)..method_rankings.len() {
            let rho = spearman(
                &method_rankings[i].ranked_alternatives,
                &method_rankings[j].ranked_alternatives,
            );
            correlations.push(rho);
            pairwise.push(PairwiseAgreement {
                method_a: method_rankings[i].method_id.clone(),
                method_b: method_rankings[j].method_id.clone(),
                spearman_rho: rho,
            });
        }
    }
    let mean_rho = if correlations.is_empty() {
        1.0
    } else {
        correlations.iter().sum::<f64>() / correlations.len() as f64
    };
    let rankings: Vec<Vec<RankedAlternative>> = method_rankings
        .iter()
        .map(|m| m.ranked_alternatives.clone())
        .collect();
    let kendall = kendall_w(&rankings);

    let mut profile_rankings = Vec::new();
    for profile in &s.weight_profiles {
        profile_rankings.push(ProfileRanking {
            profile_id: profile.profile_id.clone(),
            ranked_alternatives: rank_scores(weighted_scores(s, profile)),
        });
    }
    profile_rankings.sort_by(|a, b| a.profile_id.cmp(&b.profile_id));
    let mut reversals = 0usize;
    for i in 0..profile_rankings.len() {
        for j in (i + 1)..profile_rankings.len() {
            let a = rank_lookup(&profile_rankings[i].ranked_alternatives);
            let b = rank_lookup(&profile_rankings[j].ranked_alternatives);
            let ids: Vec<String> = a.keys().cloned().collect();
            for x in 0..ids.len() {
                for y in (x + 1)..ids.len() {
                    let da = (a[&ids[x]] - a[&ids[y]]).signum();
                    let db = (b[&ids[x]] - b[&ids[y]]).signum();
                    if da * db < 0.0 {
                        reversals += 1;
                    }
                }
            }
        }
    }

    let alternatives = sorted_alternatives(s);
    let nominal_front = pareto_front(s, false);
    let robust_front = pareto_front(s, true);
    let mut dominance = Vec::new();
    for a in &alternatives {
        for b in &alternatives {
            if a.alternative_id != b.alternative_id && robustly_dominates(a, b, &s.indicators) {
                dominance.push(DominanceRelation {
                    dominant_alternative_id: a.alternative_id.clone(),
                    dominated_alternative_id: b.alternative_id.clone(),
                });
            }
        }
    }

    let mut guardrails = Vec::new();
    for a in &alternatives {
        let results: Vec<GuardrailResult> = s
            .hard_guardrails
            .iter()
            .map(|g| {
                let m = measurement(a, &g.indicator_id);
                GuardrailResult {
                    guardrail_id: g.guardrail_id.clone(),
                    status: guardrail_status(m.value, m.standard_error, g),
                }
            })
            .collect();
        guardrails.push(AlternativeGuardrails {
            alternative_id: a.alternative_id.clone(),
            eligible: results.iter().all(|r| r.status == "clear"),
            guardrail_results: results,
        });
    }

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256: digest,
        indicator_count: s.indicators.len(),
        alternative_count: s.alternatives.len(),
        method_rankings,
        pairwise_method_agreement: pairwise,
        kendall_w: kendall,
        mean_pairwise_spearman: mean_rho,
        weight_profile_rankings: profile_rankings,
        weight_profile_pairwise_reversals: reversals,
        nominal_pareto_front_ids: nominal_front,
        robust_pareto_front_ids: robust_front,
        robust_dominance_relations: dominance,
        alternative_guardrails: guardrails,
        controls: Controls {
            no_method_promoted_as_universal_truth: true,
            guardrails_separate_from_scores: true,
            pareto_separate_from_scalar_ranks: true,
            uncertainty_overlap_abstains: true,
            permutation_invariant: true,
            no_industry_reproduction_claim: true,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output_path.exists() {
        return Err(
            io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into(),
        );
    }
    let bytes = fs::read(input_path)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest);
    let encoded = serde_json::to_vec_pretty(&report)?;
    let mut out = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)?;
    out.write_all(&encoded)?;
    out.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: assessment-agreement-audit <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(e) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("assessment agreement audit failed: {e}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!(
            "../fixtures/synthetic-assessment-agreement.json"
        ))
        .expect("fixture parses")
    }

    #[test]
    fn synthetic_methods_expose_method_and_weight_disagreement() {
        let s = scenario();
        validate(&s).expect("valid synthetic scenario");
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.method_rankings.len(), 6);
        assert_eq!(r.weight_profile_rankings.len(), 5);
        assert_eq!(r.pairwise_method_agreement.len(), 15);
        assert!(r.weight_profile_pairwise_reversals > 0);
        assert!((0.0..=1.0).contains(&r.kendall_w));
    }

    #[test]
    fn hard_guardrail_failure_cannot_be_compensated_by_method_score() {
        let r = build_report(&scenario(), "0".repeat(64));
        let a = r
            .alternative_guardrails
            .iter()
            .find(|x| x.alternative_id == "A-carbon-heavy")
            .unwrap();
        assert!(!a.eligible);
        let carbon_profile = r
            .weight_profile_rankings
            .iter()
            .find(|p| p.profile_id == "carbon_priority")
            .unwrap();
        assert_eq!(
            carbon_profile.ranked_alternatives[0].alternative_id,
            "A-carbon-heavy"
        );
    }

    #[test]
    fn strictly_dominated_known_truth_alternative_is_not_on_nominal_front() {
        let r = build_report(&scenario(), "0".repeat(64));
        assert!(
            !r.nominal_pareto_front_ids
                .contains(&"F-dominated".to_string())
        );
        assert!(
            r.robust_dominance_relations
                .iter()
                .any(|x| x.dominated_alternative_id == "F-dominated")
        );
    }

    #[test]
    fn changing_weights_does_not_change_guardrails_or_pareto() {
        let s = scenario();
        let before = build_report(&s, "0".repeat(64));
        let guard_before = before
            .alternative_guardrails
            .iter()
            .map(|x| (x.alternative_id.clone(), x.eligible))
            .collect::<BTreeMap<_, _>>();
        let front_before = before.nominal_pareto_front_ids.clone();

        let mut t = scenario();
        let carbon = t
            .weight_profiles
            .iter()
            .find(|p| p.profile_id == "carbon_priority")
            .unwrap()
            .weights
            .clone();
        t.weight_profiles
            .iter_mut()
            .find(|p| p.profile_id == "declared_expert")
            .unwrap()
            .weights = carbon;
        validate(&t).expect("valid replacement weight profile");
        let after = build_report(&t, "0".repeat(64));
        let guard_after = after
            .alternative_guardrails
            .iter()
            .map(|x| (x.alternative_id.clone(), x.eligible))
            .collect::<BTreeMap<_, _>>();
        assert_eq!(guard_before, guard_after);
        assert_eq!(front_before, after.nominal_pareto_front_ids);
        assert_ne!(
            before.method_rankings[1].ranked_alternatives[0].alternative_id,
            after.method_rankings[1].ranked_alternatives[0].alternative_id
        );
    }

    #[test]
    fn indicator_and_alternative_order_permutations_are_invariant() {
        let s = scenario();
        let before = build_report(&s, "0".repeat(64));
        let mut t = s.clone();
        t.indicators.reverse();
        t.alternatives.reverse();
        for a in &mut t.alternatives {
            a.measurements.reverse();
        }
        let after = build_report(&t, "0".repeat(64));
        assert_eq!(
            before.nominal_pareto_front_ids,
            after.nominal_pareto_front_ids
        );
        assert_eq!(
            before.robust_dominance_relations.len(),
            after.robust_dominance_relations.len()
        );
        assert_eq!(
            before.method_rankings[0]
                .ranked_alternatives
                .iter()
                .map(|x| (&x.alternative_id, x.rank))
                .collect::<Vec<_>>(),
            after.method_rankings[0]
                .ranked_alternatives
                .iter()
                .map(|x| (&x.alternative_id, x.rank))
                .collect::<Vec<_>>()
        );
        assert_eq!(
            before
                .alternative_guardrails
                .iter()
                .map(|x| (&x.alternative_id, x.eligible))
                .collect::<Vec<_>>(),
            after
                .alternative_guardrails
                .iter()
                .map(|x| (&x.alternative_id, x.eligible))
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn touching_uncertainty_intervals_do_not_create_strict_dominance() {
        let s = scenario();
        let mut a = s
            .alternatives
            .iter()
            .find(|x| x.alternative_id == "A-carbon-heavy")
            .unwrap()
            .clone();
        let mut b = s
            .alternatives
            .iter()
            .find(|x| x.alternative_id == "B-balanced")
            .unwrap()
            .clone();

        for indicator in &s.indicators {
            let am = a
                .measurements
                .iter_mut()
                .find(|m| m.indicator_id == indicator.indicator_id)
                .unwrap();
            let bm = b
                .measurements
                .iter_mut()
                .find(|m| m.indicator_id == indicator.indicator_id)
                .unwrap();
            am.standard_error = 0.1;
            bm.standard_error = 0.1;
            let gap = 1.96 * (am.standard_error + bm.standard_error);
            if indicator.direction == "higher_is_better" {
                am.value = bm.value + gap;
            } else {
                am.value = bm.value - gap;
            }
        }
        assert!(!robustly_dominates(&a, &b, &s.indicators));
    }

    #[test]
    fn unknown_direction_and_missing_measurement_fail_closed() {
        let mut s = scenario();
        s.indicators[0].direction = "maybe".into();
        assert!(validate(&s).is_err());
        let mut t = scenario();
        t.alternatives[0].measurements.pop();
        assert!(validate(&t).is_err());
    }

    #[test]
    fn weight_profiles_must_sum_to_one() {
        let mut s = scenario();
        s.weight_profiles[0].weights[0].weight = 0.99;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn uncertain_guardrail_is_not_eligible() {
        let mut s = scenario();
        let a = s
            .alternatives
            .iter_mut()
            .find(|x| x.alternative_id == "B-balanced")
            .unwrap();
        let m = a
            .measurements
            .iter_mut()
            .find(|x| x.indicator_id == "nitrate_loss")
            .unwrap();
        m.value = 39.5;
        m.standard_error = 1.0;
        let r = build_report(&s, "0".repeat(64));
        let b = r
            .alternative_guardrails
            .iter()
            .find(|x| x.alternative_id == "B-balanced")
            .unwrap();
        assert!(!b.eligible);
        assert!(b.guardrail_results.iter().any(|x| x.status == "uncertain"));
    }
}
