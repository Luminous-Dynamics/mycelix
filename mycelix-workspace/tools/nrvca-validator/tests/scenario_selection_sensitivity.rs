use serde_json::Value;
use std::collections::{BTreeSet, HashMap};

const STRESS_FIXTURE: &str =
    include_str!("../../../../fixtures/nrvca-correlated-stress-synthetic-v0.1.json");
const EXPECTED_FIXTURE: &str =
    include_str!("../../../../fixtures/nrvca-scenario-selection-sensitivity-expected-v0.1.json");
const EPS: f64 = 1e-9;

fn stress_fixture() -> Value {
    serde_json::from_str(STRESS_FIXTURE).expect("stress fixture parses")
}

fn expected_fixture() -> Value {
    serde_json::from_str(EXPECTED_FIXTURE).expect("subset known-answer fixture parses")
}

fn num(value: &Value) -> f64 {
    value.as_f64().expect("numeric field")
}

fn sid(value: &Value) -> &str {
    value["scenario_id"].as_str().expect("scenario_id")
}

fn tid(value: &Value) -> &str {
    value["strategy_id"].as_str().expect("strategy_id")
}

fn scenario_ids(stress: &Value) -> Vec<String> {
    let mut ids: Vec<String> = stress["scenario_manifest"]
        .as_array().expect("scenario manifest")
        .iter().map(|s| sid(s).to_owned()).collect();
    ids.sort();
    ids
}

fn strategy_ids(stress: &Value) -> Vec<String> {
    let mut ids: Vec<String> = stress["strategies"]
        .as_array().expect("strategies")
        .iter().map(|s| tid(s).to_owned()).collect();
    ids.sort();
    ids
}

fn find_strategy<'a>(stress: &'a Value, strategy_id: &str) -> &'a Value {
    let items = stress["strategies"].as_array().expect("strategies");
    let matches: Vec<&Value> = items.iter()
        .filter(|strategy| tid(strategy) == strategy_id).collect();
    assert_eq!(matches.len(), 1, "strategy ID must be unique: {strategy_id}");
    matches[0]
}

fn find_row<'a>(strategy: &'a Value, scenario_id: &str) -> &'a Value {
    let items = strategy["scenario_results"].as_array().expect("scenario results");
    let matches: Vec<&Value> = items.iter()
        .filter(|row| sid(row) == scenario_id).collect();
    assert_eq!(matches.len(), 1,
        "{} must have one outcome for {scenario_id}", tid(strategy));
    matches[0]
}

fn validate_matrix(stress: &Value) -> Result<(), String> {
    let scenarios = stress["scenario_manifest"].as_array()
        .ok_or_else(|| "scenario_manifest must be an array".to_owned())?;
    let strategies = stress["strategies"].as_array()
        .ok_or_else(|| "strategies must be an array".to_owned())?;
    let scenario_id_list: Vec<&str> = scenarios.iter().map(sid).collect();
    let unique_scenarios: BTreeSet<&str> = scenario_id_list.iter().copied().collect();
    if unique_scenarios.len() != scenario_id_list.len() {
        return Err("duplicate scenario IDs".to_owned());
    }
    let strategy_id_list: Vec<&str> = strategies.iter().map(tid).collect();
    let unique_strategies: BTreeSet<&str> = strategy_id_list.iter().copied().collect();
    if unique_strategies.len() != strategy_id_list.len() {
        return Err("duplicate strategy IDs".to_owned());
    }
    for strategy in strategies {
        let rows = strategy["scenario_results"].as_array()
            .ok_or_else(|| format!("{} missing scenario_results", tid(strategy)))?;
        let row_ids: Vec<&str> = rows.iter().map(sid).collect();
        let unique_rows: BTreeSet<&str> = row_ids.iter().copied().collect();
        if unique_rows.len() != row_ids.len() {
            return Err(format!("duplicate outcome cell for {}", tid(strategy)));
        }
        if unique_rows != unique_scenarios {
            return Err(format!("incomplete scenario matrix for {}", tid(strategy)));
        }
    }
    Ok(())
}

fn row_passes(stress: &Value, strategy: &Value, row: &Value) -> bool {
    let policy = &stress["scoring_policy"];
    let base = num(&strategy["base_cost"]);
    let total = num(&row["total_cost"]);
    base <= num(&policy["primary_budget"]) + EPS
        && total >= base - EPS
        && total - base <= num(&policy["contingency_reserve_cap"]) + EPS
        && total <= num(&policy["total_cost_upper_bound"]) + EPS
        && num(&row["service_units_lower_bound"])
            >= num(&policy["min_service_units_lower_bound"]) - EPS
        && num(&row["underserved_service_units_lower_bound"])
            >= num(&policy["min_underserved_service_units_lower_bound"]) - EPS
        && num(&row["reliability_lower_bound"])
            >= num(&policy["min_reliability_lower_bound"]) - EPS
        && num(&row["emissions_tonnes_co2e_upper_bound"])
            <= num(&policy["max_emissions_tonnes_co2e_upper_bound"]) + EPS
}

#[derive(Debug, Clone)]
struct Score {
    strategy_id: String,
    pass_count: usize,
    scenario_count: usize,
    fraction: f64,
}

fn score_subset(stress: &Value, included_scenarios: &[String]) -> Vec<Score> {
    assert!(!included_scenarios.is_empty(), "empty scenario set is not a robustness score");
    let mut scores = Vec::new();
    for id in strategy_ids(stress) {
        let strategy = find_strategy(stress, &id);
        let pass_count = included_scenarios.iter()
            .filter(|scenario| row_passes(stress, strategy, find_row(strategy, scenario.as_str())))
            .count();
        scores.push(Score {
            strategy_id: id,
            pass_count,
            scenario_count: included_scenarios.len(),
            fraction: pass_count as f64 / included_scenarios.len() as f64,
        });
    }
    scores
}

fn score_for<'a>(scores: &'a [Score], strategy_id: &str) -> &'a Score {
    scores.iter().find(|score| score.strategy_id == strategy_id)
        .unwrap_or_else(|| panic!("missing score for {strategy_id}"))
}

fn strings(value: &Value) -> Vec<String> {
    let mut ids: Vec<String> = value.as_array().expect("string array")
        .iter().map(|x| x.as_str().expect("string item").to_owned()).collect();
    ids.sort();
    ids
}

fn top_ids(scores: &[Score]) -> Vec<String> {
    let best = scores.iter().map(|score| score.fraction).fold(f64::NEG_INFINITY, f64::max);
    let mut ids: Vec<String> = scores.iter()
        .filter(|score| (score.fraction - best).abs() <= EPS)
        .map(|score| score.strategy_id.clone()).collect();
    ids.sort();
    ids
}

fn assert_fraction(actual: f64, expected: f64) {
    assert!((actual - expected).abs() <= 1e-9,
        "actual fraction {actual} differs from expected {expected}");
}

fn subset_key(ids: &[String]) -> String {
    format!("set:{}", ids.join("|"))
}

fn exhaustive_subsets(ids: &[String]) -> Vec<(Vec<String>, Vec<String>)> {
    assert!(ids.len() < usize::BITS as usize, "too many scenarios for exact bitmask enumeration");
    let total = 1usize << ids.len();
    let mut result = Vec::with_capacity(total - 1);
    for mask in 1..total {
        let mut included = Vec::new();
        let mut excluded = Vec::new();
        for (index, id) in ids.iter().enumerate() {
            if mask & (1usize << index) != 0 {
                included.push(id.clone());
            } else {
                excluded.push(id.clone());
            }
        }
        result.push((included, excluded));
    }
    result
}

#[test]
fn all_nonempty_subsets_have_exact_frozen_scores_and_disclosed_denominators() {
    let stress = stress_fixture();
    let expected = expected_fixture();
    validate_matrix(&stress).expect("stress matrix must be complete");

    let scenario_ids = scenario_ids(&stress);
    let strategies = strategy_ids(&stress);
    assert_eq!(scenario_ids.len(), 5);
    assert_eq!(strategies.len(), 3);
    let subsets = exhaustive_subsets(&scenario_ids);
    assert_eq!(subsets.len(), 31);
    assert_eq!(expected["subset_generation"]["expected_subset_count"].as_u64(), Some(31));
    assert_eq!(expected["subset_generation"]["probabilities_assigned"], false);
    assert_eq!(expected["subset_generation"]["subset_weights_used"], false);

    let expected_subsets = expected["expected_subsets"].as_array().expect("expected subsets");
    assert_eq!(expected_subsets.len(), subsets.len());
    let expected_by_key: HashMap<String, &Value> = expected_subsets.iter()
        .map(|item| (item["subset_id"].as_str().expect("subset_id").to_owned(), item))
        .collect();
    assert_eq!(expected_by_key.len(), subsets.len(), "subset keys must be unique");

    for (included, excluded) in &subsets {
        let key = subset_key(included);
        let frozen = expected_by_key.get(&key)
            .unwrap_or_else(|| panic!("missing subset known answer: {key}"));
        assert_eq!(strings(&frozen["scenario_ids_included"]), included.clone());
        assert_eq!(strings(&frozen["scenario_ids_excluded"]), excluded.clone());
        assert_eq!(frozen["scenario_count"].as_u64(), Some(included.len() as u64));

        let scores = score_subset(&stress, included);
        let expected_scores = frozen["strategy_results"].as_array().expect("strategy results");
        assert_eq!(expected_scores.len(), strategies.len());
        for expected_score in expected_scores {
            let id = expected_score["strategy_id"].as_str().expect("strategy id");
            let actual = score_for(&scores, id);
            assert_eq!(actual.pass_count, expected_score["pass_count"].as_u64().unwrap() as usize);
            assert_eq!(actual.scenario_count, expected_score["scenario_count"].as_u64().unwrap() as usize);
            assert_fraction(actual.fraction, num(&expected_score["robust_pass_fraction"]));
            assert_eq!(actual.scenario_count, included.len(), "denominator must be explicit");
            assert_eq!(actual.pass_count + (actual.scenario_count - actual.pass_count), included.len());
        }
        assert_eq!(top_ids(&scores), strings(&frozen["top_rank_strategy_ids"]));
    }
}

#[test]
fn score_ranges_and_leave_one_out_answers_are_reproducible() {
    let stress = stress_fixture();
    let expected = expected_fixture();
    let ids = scenario_ids(&stress);
    let subsets = exhaustive_subsets(&ids);
    let mut every_score: HashMap<String, Vec<f64>> = HashMap::new();
    let mut top_count: HashMap<String, usize> = HashMap::new();
    let mut sole_top_count: HashMap<String, usize> = HashMap::new();
    let mut ranking_order_holds = true;

    for (included, _) in &subsets {
        let scores = score_subset(&stress, included);
        for score in &scores {
            every_score.entry(score.strategy_id.clone()).or_default().push(score.fraction);
            if top_ids(&scores).contains(&score.strategy_id) {
                *top_count.entry(score.strategy_id.clone()).or_default() += 1;
            }
        }
        let top = top_ids(&scores);
        if top.len() == 1 {
            *sole_top_count.entry(top[0].clone()).or_default() += 1;
        }
        let adaptive = score_for(&scores, "adaptive_staged").fraction;
        let equity = score_for(&scores, "equity_static").fraction;
        let output = score_for(&scores, "output_only_static").fraction;
        if adaptive + EPS < equity || equity + EPS < output {
            ranking_order_holds = false;
        }
    }

    assert_eq!(expected["top_rank_ordering"]["observed_in_all_subsets"], ranking_order_holds);
    for record in expected["strategy_score_ranges"].as_array().unwrap() {
        let id = record["strategy_id"].as_str().unwrap();
        let values = every_score.get(id).unwrap();
        let min = values.iter().copied().fold(f64::INFINITY, f64::min);
        let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let distinct: BTreeSet<String> = values.iter().map(|x| format!("{x:.12}")).collect();
        assert_fraction(min, num(&record["min_robust_pass_fraction"]));
        assert_fraction(max, num(&record["max_robust_pass_fraction"]));
        assert_eq!(distinct.len(), record["distinct_fraction_count"].as_u64().unwrap() as usize);
        assert_eq!(*top_count.get(id).unwrap_or(&0), record["subset_count_including_top_tie"].as_u64().unwrap() as usize);
        assert_eq!(*sole_top_count.get(id).unwrap_or(&0), record["subset_count_as_sole_top"].as_u64().unwrap() as usize);
    }

    let leave_one_out = expected["leave_one_out"].as_array().unwrap();
    assert_eq!(leave_one_out.len(), ids.len());
    for excluded_record in leave_one_out {
        let excluded = excluded_record["excluded_scenario_id"].as_str().unwrap();
        let subset = ids.iter().filter(|id| id.as_str() != excluded).cloned().collect::<Vec<_>>();
        assert_eq!(strings(&excluded_record["remaining_scenario_ids"]), subset);
        assert_eq!(excluded_record["scenario_count"].as_u64(), Some(subset.len() as u64));
        let scores = score_subset(&stress, &subset);
        for expected_score in excluded_record["strategy_results"].as_array().unwrap() {
            let actual = score_for(&scores, expected_score["strategy_id"].as_str().unwrap());
            assert_eq!(actual.pass_count, expected_score["pass_count"].as_u64().unwrap() as usize);
            assert_fraction(actual.fraction, num(&expected_score["robust_pass_fraction"]));
        }
    }
}

#[test]
fn full_set_reproduces_existing_robust_pass_counts_without_changing_them() {
    let stress = stress_fixture();
    let expected = expected_fixture();
    let ids = scenario_ids(&stress);
    let scores = score_subset(&stress, &ids);
    for frozen in stress["expected_summaries"].as_array().unwrap() {
        let id = frozen["strategy_id"].as_str().unwrap();
        let score = score_for(&scores, id);
        assert_eq!(score.pass_count, frozen["robust_pass_count"].as_u64().unwrap() as usize);
        assert_eq!(score.scenario_count, frozen["scenario_count"].as_u64().unwrap() as usize);
        assert_fraction(score.fraction, num(&frozen["robust_pass_fraction"]));
    }
    assert_eq!(expected["expected_subsets"].as_array().unwrap().len(), 31);
}

#[test]
fn scenario_and_strategy_order_or_descriptive_labels_do_not_change_subset_scores() {
    let stress = stress_fixture();
    let ids = scenario_ids(&stress);
    let subset = ids.iter().filter(|id| id.as_str() != "prolonged-repeat-disruption")
        .cloned().collect::<Vec<_>>();
    let before = score_subset(&stress, &subset);

    let mut changed = stress.clone();
    changed["scenario_manifest"].as_array_mut().unwrap().reverse();
    changed["strategies"].as_array_mut().unwrap().reverse();
    for strategy in changed["strategies"].as_array_mut().unwrap() {
        let id = tid(strategy).to_owned();
        strategy["label"] = serde_json::json!(format!("display-only-{id}"));
        strategy["scenario_results"].as_array_mut().unwrap().reverse();
    }
    let after = score_subset(&changed, &subset);
    for score in &before {
        let other = score_for(&after, &score.strategy_id);
        assert_eq!(score.pass_count, other.pass_count);
        assert_eq!(score.scenario_count, other.scenario_count);
        assert_fraction(score.fraction, other.fraction);
    }
}

#[test]
fn added_passing_or_failing_case_changes_only_the_disclosed_subset_metric() {
    let stress = stress_fixture();
    let subset = vec!["nominal-operations".to_owned()];
    let base = score_subset(&stress, &subset);
    let adaptive_before = score_for(&base, "adaptive_staged").fraction;
    assert_fraction(adaptive_before, 1.0);

    let mut altered = stress.clone();
    let adaptive = altered["strategies"].as_array_mut().unwrap()
        .iter_mut().find(|s| tid(s) == "adaptive_staged").unwrap();
    let nominal = adaptive["scenario_results"].as_array_mut().unwrap()
        .iter_mut().find(|r| sid(r) == "nominal-operations").unwrap();
    nominal["service_units_lower_bound"] = serde_json::json!(0);
    let failing = vec!["nominal-operations".to_owned()];
    let failed_score = score_subset(&altered, &failing);
    assert_fraction(score_for(&failed_score, "adaptive_staged").fraction, 0.0);

    let original_with_pass = score_subset(&stress, &vec![
        "nominal-operations".to_owned(),
        "prolonged-repeat-disruption".to_owned()
    ]);
    let adaptive = score_for(&original_with_pass, "adaptive_staged");
    assert_eq!(adaptive.pass_count, 1);
    assert_eq!(adaptive.scenario_count, 2);
    assert_fraction(adaptive.fraction, 0.5);
    // Adding a passing case to a failing-only set raises 0/1 to 1/2.
    let fail_only = vec!["prolonged-repeat-disruption".to_owned()];
    let fail_before = score_subset(&stress, &fail_only);
    assert_fraction(score_for(&fail_before, "adaptive_staged").fraction, 0.0);
    let with_pass = vec!["nominal-operations".to_owned(), "prolonged-repeat-disruption".to_owned()];
    let after_pass = score_subset(&stress, &with_pass);
    assert!(score_for(&after_pass, "adaptive_staged").fraction
        > score_for(&fail_before, "adaptive_staged").fraction);
}
