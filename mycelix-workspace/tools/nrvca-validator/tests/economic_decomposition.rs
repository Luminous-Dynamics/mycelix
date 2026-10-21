use serde_json::Value;

const FIXTURE: &str =
    include_str!("../../../../fixtures/nrvca-economic-decomposition-golden-v0.1.json");

fn fixture() -> Value {
    serde_json::from_str(FIXTURE).expect("economic-decomposition fixture parses")
}

fn case_by_id<'a>(root: &'a Value, case_id: &str) -> &'a Value {
    root["cases"]
        .as_array()
        .expect("cases is an array")
        .iter()
        .find(|case| case["case_id"].as_str() == Some(case_id))
        .unwrap_or_else(|| panic!("missing case {case_id}"))
}

fn number(value: &Value) -> f64 {
    value.as_f64().expect("expected finite numeric fixture field")
}

fn close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 1e-7,
        "actual {actual} differs from expected {expected}"
    );
}

/// Independent closed-form oracle for the tiny model only.
/// Demand: P = a - bQ; marginal cost: MC = c + dQ.
/// This is deliberately outside the production NRVCA estimate path.
fn linear_market(a: f64, b: f64, c: f64, d: f64) -> Value {
    let qc = (a - c) / (b + d);
    let pc = a - b * qc;
    let cs_c = 0.5 * (a - pc) * qc;
    let ps_c = pc * qc - c * qc - 0.5 * d * qc * qc;
    let tc = cs_c + ps_c;

    let qm = (a - c) / (2.0 * b + d);
    let pm = a - b * qm;
    let cs_m = 0.5 * (a - pm) * qm;
    let ps_m = pm * qm - c * qm - 0.5 * d * qm * qm;
    let tm = cs_m + ps_m;

    serde_json::json!({
        "competitive": {
            "quantity": qc,
            "price": pc,
            "consumer_surplus": cs_c,
            "producer_surplus": ps_c,
            "total_surplus": tc
        },
        "monopoly": {
            "quantity": qm,
            "price": pm,
            "consumer_surplus": cs_m,
            "producer_surplus": ps_m,
            "total_surplus": tm,
            "price_transfer_on_retained_units": (pm - pc) * qm,
            "lost_consumer_surplus_vs_competition": cs_c - cs_m,
            "producer_surplus_gain_vs_competition": ps_m - ps_c,
            "deadweight_loss_vs_competition": tc - tm
        }
    })
}

#[test]
fn pure_transfer_closes_counterparties_without_becoming_resource_loss() {
    let root = fixture();
    let case = case_by_id(&root, "pure-domestic-transfer");
    let entries = case["counterparty_entries"].as_array().unwrap();
    let signed_sum: f64 = entries.iter().map(|entry| number(&entry["signed_amount"])).sum();

    close(signed_sum, 0.0);
    close(number(&case["expected"]["consolidated_net_transfer"]), 0.0);
    close(number(&case["expected"]["distributional_incidence"]), 100.0);
    close(number(&case["expected"]["observed_real_resource_cost"]), 0.0);
    close(
        number(&case["expected"]["net_resource_loss_in_this_isolated_case"]),
        0.0,
    );
    assert!(case["expected"]["deadweight_loss"].is_null());
    assert_eq!(
        case["expected"]["must_not_infer_general_welfare_effect_from_transfer_alone"],
        true
    );
}

#[test]
fn independent_market_oracle_matches_known_answers_and_separates_transfer_from_dwl() {
    let root = fixture();
    let case = case_by_id(&root, "linear-market-power");
    let a = number(&case["model"]["inverse_demand"]["intercept"]);
    let b = number(&case["model"]["inverse_demand"]["slope"]);
    let c = number(&case["model"]["marginal_cost"]["intercept"]);
    let d = number(&case["model"]["marginal_cost"]["slope"]);
    let actual = linear_market(a, b, c, d);
    let expected = &case["expected"];

    for section in ["competitive", "monopoly"] {
        for field in expected[section].as_object().unwrap().keys() {
            close(number(&actual[section][field]), number(&expected[section][field]));
        }
    }

    let monopoly = &actual["monopoly"];
    close(
        number(&monopoly["lost_consumer_surplus_vs_competition"]),
        number(&monopoly["producer_surplus_gain_vs_competition"])
            + number(&monopoly["deadweight_loss_vs_competition"]),
    );
    close(
        number(&monopoly["deadweight_loss_vs_competition"]),
        number(&actual["competitive"]["total_surplus"])
            - number(&monopoly["total_surplus"]),
    );
    assert!(
        (number(&monopoly["price_transfer_on_retained_units"])
            - number(&monopoly["deadweight_loss_vs_competition"]))
            .abs()
            > 1.0,
        "price-transfer incidence must not be mislabeled as deadweight loss"
    );
}

#[test]
fn markup_and_rent_capture_do_not_auto_generate_social_loss() {
    let root = fixture();
    let markup = case_by_id(&root, "markup-with-fixed-cost-recovery");
    close(
        number(&markup["observed"]["revenue"]),
        number(&markup["units_sold"]) * number(&markup["price_per_unit"]),
    );
    close(
        number(&markup["observed"]["variable_cost"]),
        number(&markup["units_sold"])
            * number(&markup["marginal_variable_cost_per_unit"]),
    );
    close(
        number(&markup["observed"]["operating_result_after_fixed_cost"]),
        number(&markup["observed"]["contribution_after_variable_cost"])
            - number(&markup["fixed_rnd_and_capital_recovery_cost"]),
    );
    assert!(markup["expected"]["economic_rent"].is_null());
    assert!(markup["expected"]["deadweight_loss"].is_null());
    assert!(markup["expected"]["social_loss_from_markup_alone"].is_null());

    let rent = case_by_id(&root, "rent-capture-vs-real-resource-cost");
    close(number(&rent["captured_transfer_or_rent"]), 1000.0);
    close(number(&rent["observed_real_resources_spent_on_capture"]), 120.0);
    assert!(rent["deadweight_loss"].is_null());
    assert!(rent["external_damage"].is_null());
    assert!(rent["expected"]["automatic_total_social_loss"].is_null());
}

#[test]
fn shadow_price_sensitivity_changes_valuation_not_physical_measurement() {
    let root = fixture();
    let case = case_by_id(&root, "physical-externality-sensitivity");
    let quantity = number(&case["physical_measurement"]["quantity"]);
    let scenarios = case["valuation_scenarios"].as_array().unwrap();
    assert_eq!(scenarios.len(), 3);
    for scenario in scenarios {
        close(
            quantity * number(&scenario["shadow_price_zar_per_tonne"]),
            number(&scenario["modelled_damage_zar"]),
        );
    }
    assert_eq!(case["physical_measurement"]["quantity"], 200);
    assert_eq!(
        case["expected"]["monetary_damage_is_modelled_not_observed"],
        true
    );

    // Metamorphic guard: adding/changing a market-power label must not alter the
    // physical-to-valuation calculation because the formula has no such input.
    let before: Vec<f64> = scenarios
        .iter()
        .map(|s| quantity * number(&s["shadow_price_zar_per_tonne"]))
        .collect();
    let mut relabelled = case.clone();
    relabelled["irrelevant_markup_label"] = serde_json::json!("very high");
    let q_after = number(&relabelled["physical_measurement"]["quantity"]);
    let after: Vec<f64> = relabelled["valuation_scenarios"]
        .as_array()
        .unwrap()
        .iter()
        .map(|s| q_after * number(&s["shadow_price_zar_per_tonne"]))
        .collect();
    assert_eq!(before, after);
}

#[test]
fn public_service_without_a_price_remains_unknown_not_zero() {
    let root = fixture();
    let case = case_by_id(&root, "public-service-without-market-price");
    assert!(case["observed_market_price"].is_null());
    assert!(case["social_value"].is_null());
    assert_eq!(case["expected"]["value_status"], "unknown");
    assert_eq!(case["expected"]["must_not_impute_zero_value"], true);
    assert_eq!(case["expected"]["must_not_invent_price_from_missing_market_price"], true);
    assert_eq!(case["expected"]["must_report_delivered_units_separately"], true);
    close(number(&case["service_units_delivered"]), 500.0);
}
