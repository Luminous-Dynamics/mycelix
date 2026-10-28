use serde_json::{json, Value};
use std::{
    collections::BTreeSet,
    env, fs,
    path::PathBuf,
    process::ExitCode,
};

const EMBEDDED_FIXTURE: &str = include_str!(
    "../../../docs/research/fixtures/CECB_HOUSING_KNOWN_TRUTH_V0_1.json"
);

const EXPECTED_SCHEMA: &str = "cecb.housing.known_truth.v0.1";
const EXPECTED_CASES: &[&str] = &[
    "H001_FIXED_CAPACITY_CEILING",
    "H002_DEBT_FINANCING_IS_NOT_CAPACITY",
    "H003_GRANT_HAS_A_REAL_RESOURCE_COST",
    "H004_STALE_LISTING_IS_NOT_SAFE_CAPACITY",
    "H005_DEFERRED_MAINTENANCE_MOVES_COST_FORWARD",
    "H006_INTERNAL_MUTUAL_CREDIT_CANNOT_SETTLE_EXTERNAL_INPUT",
    "H007_REFINANCING_IS_NOT_NET_NEW_CAPITAL",
    "H008_RENT_TRANSFER_DOES_NOT_PROVE_NEW_CAPACITY",
    "H009_UNIT_COUNT_DOES_NOT_CAPTURE_ACCESSIBILITY_FIT",
    "H010_MIRRORED_SOURCES_ARE_NOT_INDEPENDENT_CONFIRMATION",
];

#[derive(Debug)]
struct Cli {
    fixture_path: Option<PathBuf>,
    policy_view_out: Option<PathBuf>,
}

fn main() -> ExitCode {
    let cli = match parse_args() {
        Ok(Some(cli)) => cli,
        Ok(None) => return ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("{message}");
            return ExitCode::from(2);
        }
    };

    let input = match &cli.fixture_path {
        Some(path) => match fs::read_to_string(path) {
            Ok(input) => input,
            Err(error) => {
                eprintln!("cannot read fixture {}: {error}", path.display());
                return ExitCode::from(2);
            }
        },
        None => EMBEDDED_FIXTURE.to_owned(),
    };

    let (report, passed) = match validate_fixture(&input) {
        Ok(result) => result,
        Err(message) => {
            eprintln!("fixture validation could not start: {message}");
            return ExitCode::from(2);
        }
    };

    match serde_json::to_string_pretty(&report) {
        Ok(output) => println!("{output}"),
        Err(error) => {
            eprintln!("cannot serialize validation report: {error}");
            return ExitCode::from(2);
        }
    }

    if passed {
        if let Some(path) = &cli.policy_view_out {
            let view = match make_policy_view(&input) {
                Ok(view) => view,
                Err(error) => {
                    eprintln!("cannot build candidate-visible policy view: {error}");
                    return ExitCode::from(2);
                }
            };
            let output = match serde_json::to_string_pretty(&view) {
                Ok(output) => output,
                Err(error) => {
                    eprintln!("cannot serialize candidate-visible policy view: {error}");
                    return ExitCode::from(2);
                }
            };
            if let Err(error) = fs::write(path, format!("{output}\n")) {
                eprintln!("cannot write policy view {}: {error}", path.display());
                return ExitCode::from(2);
            }
        }
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

fn parse_args() -> Result<Option<Cli>, String> {
    let mut cli = Cli {
        fixture_path: None,
        policy_view_out: None,
    };
    let args: Vec<String> = env::args().skip(1).collect();
    let mut index = 0;

    while index < args.len() {
        match args[index].as_str() {
            "--help" | "-h" => {
                println!(
                    "cecb-housing-fixture-check [--fixture PATH] [--policy-view-out PATH]\n\
                     Validates the v0.1 synthetic known-truth fixture. By default it uses the\n\
                     repository fixture embedded at compile time. The policy view contains only\n\
                     candidate-visible observations and excludes world/oracle truth."
                );
                return Ok(None);
            }
            "--fixture" => {
                index += 1;
                let value = args
                    .get(index)
                    .ok_or("--fixture requires a path")?;
                cli.fixture_path = Some(PathBuf::from(value));
            }
            "--policy-view-out" => {
                index += 1;
                let value = args
                    .get(index)
                    .ok_or("--policy-view-out requires a path")?;
                cli.policy_view_out = Some(PathBuf::from(value));
            }
            other => return Err(format!("unknown argument: {other}; use --help")),
        }
        index += 1;
    }
    Ok(Some(cli))
}

fn validate_fixture(input: &str) -> Result<(Value, bool), String> {
    let root: Value = serde_json::from_str(input)
        .map_err(|error| format!("invalid JSON: {error}"))?;

    let schema = root
        .get("schema_version")
        .and_then(Value::as_str)
        .ok_or("missing string schema_version")?;
    if schema != EXPECTED_SCHEMA {
        return Err(format!(
            "unsupported schema_version {schema:?}; expected {EXPECTED_SCHEMA:?}"
        ));
    }

    let cases = root
        .get("cases")
        .and_then(Value::as_array)
        .ok_or("missing cases array")?;

    let mut global_violations = Vec::<String>::new();
    if cases.len() != EXPECTED_CASES.len() {
        global_violations.push(format!(
            "fixture contains {} cases; schema v0.1 requires {}",
            cases.len(),
            EXPECTED_CASES.len()
        ));
    }

    let mut ids = BTreeSet::<String>::new();
    for case in cases {
        let id = case
            .get("case_id")
            .and_then(Value::as_str)
            .unwrap_or("<missing-case-id>")
            .to_owned();
        if !ids.insert(id.clone()) {
            global_violations.push(format!("duplicate case_id {id}"));
        }
    }

    for expected in EXPECTED_CASES {
        if !ids.contains(*expected) {
            global_violations.push(format!("required case missing: {expected}"));
        }
    }
    for id in &ids {
        if !EXPECTED_CASES.contains(&id.as_str()) {
            global_violations.push(format!("unexpected case_id: {id}"));
        }
    }

    let mut case_results = Vec::<Value>::new();
    let mut all_cases_passed = true;
    for case in cases {
        let id = case
            .get("case_id")
            .and_then(Value::as_str)
            .unwrap_or("<missing-case-id>");
        let violations = validate_case(case);
        let status = if violations.is_empty() { "PASS" } else { "FAIL" };
        if !violations.is_empty() {
            all_cases_passed = false;
        }
        case_results.push(json!({
            "case_id": id,
            "status": status,
            "violation_count": violations.len(),
            "violations": violations,
        }));
    }

    let passed = global_violations.is_empty() && all_cases_passed;
    let pass_count = case_results
        .iter()
        .filter(|result| result["status"] == "PASS")
        .count();
    let report = json!({
        "report_schema": "cecb.housing.fixture_validation_report.v0.1",
        "fixture_schema": schema,
        "status": if passed { "PASS" } else { "FAIL" },
        "case_count": cases.len(),
        "case_pass_count": pass_count,
        "case_fail_count": cases.len().saturating_sub(pass_count),
        "global_violations": global_violations,
        "case_results": case_results,
        "claim_ceiling": concat!(
            "Validates fixture consistency only; it neither executes policies nor proves real-world impact. ",
            "economic policies and is not evidence of real-world impact."
        )
    });
    Ok((report, passed))
}

fn validate_case(case: &Value) -> Vec<String> {
    let mut errors = Vec::<String>::new();
    let id = case
        .get("case_id")
        .and_then(Value::as_str)
        .unwrap_or("<missing-case-id>");

    if id == "<missing-case-id>" {
        errors.push("case_id must be a non-empty string".to_owned());
    }
    if case.get("purpose").and_then(Value::as_str).is_none() {
        errors.push("purpose must be a string".to_owned());
    }
    if case.get("world").and_then(Value::as_object).is_none() {
        errors.push("world must be an object".to_owned());
    }
    if case
        .get("policy_observation")
        .and_then(Value::as_object)
        .is_none()
    {
        errors.push("policy_observation must be an object".to_owned());
    }
    if case.get("oracle_facts").and_then(Value::as_object).is_none() {
        errors.push("oracle_facts must be an object".to_owned());
    }
    match case.get("invariants").and_then(Value::as_array) {
        Some(invariants) if !invariants.is_empty() => {}
        _ => errors.push("invariants must be a non-empty array".to_owned()),
    }

    if let Some(observation) = case.get("policy_observation") {
        for forbidden in [
            "world",
            "oracle_facts",
            "true_safe",
            "true_suitable_for_registered_need",
            "true_safe_suitable_capacity",
            "maximum_households_housed_by_end",
            "minimum_unmet_households_by_end",
            "independent_confirmation_count",
        ] {
            if contains_key_recursive(observation, forbidden) {
                errors.push(format!(
                    "candidate-visible policy_observation leaks evaluator-only field {forbidden:?}"
                ));
            }
        }
    }

    // Catch future oracle fields automatically, not only names enumerated above.
    if let (Some(observation), Some(oracle_facts)) =
        (case.get("policy_observation"), case.get("oracle_facts"))
    {
        let mut observation_keys = BTreeSet::<String>::new();
        let mut oracle_keys = BTreeSet::<String>::new();
        collect_object_keys(observation, &mut observation_keys);
        collect_object_keys(oracle_facts, &mut oracle_keys);
        for leaked_key in observation_keys.intersection(&oracle_keys) {
            errors.push(format!(
                "candidate-visible observation duplicates evaluator-only oracle key {leaked_key:?}"
            ));
        }
    }

    match id {
        "H001_FIXED_CAPACITY_CEILING" => validate_h001(case, &mut errors),
        "H002_DEBT_FINANCING_IS_NOT_CAPACITY" => validate_h002(case, &mut errors),
        "H003_GRANT_HAS_A_REAL_RESOURCE_COST" => validate_h003(case, &mut errors),
        "H004_STALE_LISTING_IS_NOT_SAFE_CAPACITY" => validate_h004(case, &mut errors),
        "H005_DEFERRED_MAINTENANCE_MOVES_COST_FORWARD" => validate_h005(case, &mut errors),
        "H006_INTERNAL_MUTUAL_CREDIT_CANNOT_SETTLE_EXTERNAL_INPUT" => {
            validate_h006(case, &mut errors)
        }
        "H007_REFINANCING_IS_NOT_NET_NEW_CAPITAL" => validate_h007(case, &mut errors),
        "H008_RENT_TRANSFER_DOES_NOT_PROVE_NEW_CAPACITY" => validate_h008(case, &mut errors),
        "H009_UNIT_COUNT_DOES_NOT_CAPTURE_ACCESSIBILITY_FIT" => validate_h009(case, &mut errors),
        "H010_MIRRORED_SOURCES_ARE_NOT_INDEPENDENT_CONFIRMATION" => {
            validate_h010(case, &mut errors)
        }
        _ => errors.push(format!("unknown v0.1 case_id {id:?}")),
    }
    errors
}

fn validate_h001(case: &Value, errors: &mut Vec<String>) {
    expect_u64(case, &["world", "households_with_unmet_need"], 3, "household demand", errors);
    expect_u64(
        case,
        &["world", "safe_suitable_vacant_units"],
        2,
        "physical housing capacity",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "maximum_households_housed_by_end"],
        2,
        "maximum housed households",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "minimum_unmet_households_by_end"],
        1,
        "minimum unmet households",
        errors,
    );
    expect_u64(
        case,
        &["world", "new_construction_completions"],
        0,
        "completed construction",
        errors,
    );

    if let (Some(demand), Some(housed), Some(unmet)) = (
        u64_at(case, &["world", "households_with_unmet_need"]),
        u64_at(case, &["oracle_facts", "maximum_households_housed_by_end"]),
        u64_at(case, &["oracle_facts", "minimum_unmet_households_by_end"]),
    ) {
        if housed.saturating_add(unmet) != demand {
            errors.push("housed + unmet must equal household demand".to_owned());
        }
    }
    if let (Some(capacity), Some(housed)) = (
        u64_at(case, &["world", "safe_suitable_vacant_units"]),
        u64_at(case, &["oracle_facts", "maximum_households_housed_by_end"]),
    ) {
        if housed > capacity {
            errors.push("housed households exceed safe, suitable physical capacity".to_owned());
        }
    }
}

fn validate_h002(case: &Value, errors: &mut Vec<String>) {
    for (path, expected, label) in [
        (&["world", "initial_safe_suitable_units"][..], 2, "initial capacity"),
        (
            &["world", "construction_project", "safe_suitable_units_added_on_completion"][..],
            1,
            "completed units",
        ),
        (&["world", "construction_project", "completion_month"][..], 2, "completion month"),
        (&["world", "financing_event", "principal_advanced_cu"][..], 10, "loan principal"),
        (&["world", "financing_event", "liability_created_cu"][..], 10, "created liability"),
        (&["oracle_facts", "safe_suitable_units_at_month_0"][..], 2, "capacity at origination"),
        (
            &["oracle_facts", "safe_suitable_units_before_completion"][..],
            2,
            "capacity before completion",
        ),
        (
            &["oracle_facts", "safe_suitable_units_after_month_2_completion"][..],
            3,
            "capacity after completion",
        ),
        (
            &["oracle_facts", "physical_capacity_change_at_origination"][..],
            0,
            "capacity change at loan origination",
        ),
        (
            &["oracle_facts", "physical_capacity_change_on_completion"][..],
            1,
            "capacity change on completion",
        ),
        (
            &["oracle_facts", "liability_immediately_after_origination_cu"][..],
            10,
            "liability immediately after origination",
        ),
    ] {
        expect_u64(case, path, expected, label, errors);
    }

    if let (Some(before), Some(added), Some(after)) = (
        u64_at(case, &["oracle_facts", "safe_suitable_units_before_completion"]),
        u64_at(case, &["world", "construction_project", "safe_suitable_units_added_on_completion"]),
        u64_at(case, &["oracle_facts", "safe_suitable_units_after_month_2_completion"]),
    ) {
        if before.saturating_add(added) != after {
            errors.push(
                "completed capacity must equal prior capacity plus units added".to_owned(),
            );
        }
    }
}

fn validate_h003(case: &Value, errors: &mut Vec<String>) {
    expect_u64(
        case,
        &["world", "renovation_resource_requirement"],
        5,
        "renovation resource requirement",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "physical_resource_consumed_if_completed"],
        5,
        "physical resources consumed",
        errors,
    );
    // The fixture expresses initial and final capacity through the renovation event.
    expect_u64(case, &["world", "initial_safe_suitable_units"], 1, "initial safe capacity", errors);
    expect_u64(
        case,
        &["oracle_facts", "safe_suitable_units_before_completion"],
        1,
        "safe capacity before renovation completion",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "safe_suitable_units_after_completion"],
        2,
        "safe capacity after renovation completion",
        errors,
    );
    expect_i64(
        case,
        &["oracle_facts", "grantor_budget_change_cu"],
        -5,
        "grantor budget change",
        errors,
    );
    expect_i64(
        case,
        &["oracle_facts", "recipient_funding_change_cu"],
        5,
        "recipient funding change",
        errors,
    );
    expect_u64(case, &["world", "grant_cu"], 5, "grant amount", errors);
    if let (Some(grantor), Some(recipient)) = (
        i64_at(case, &["oracle_facts", "grantor_budget_change_cu"]),
        i64_at(case, &["oracle_facts", "recipient_funding_change_cu"]),
    ) {
        if grantor + recipient != 0 {
            errors.push("grant transfer must reconcile between grantor and recipient".to_owned());
        }
    }
}

fn validate_h004(case: &Value, errors: &mut Vec<String>) {
    expect_u64(
        case,
        &["policy_observation", "registry_available_listing_count"],
        3,
        "reported listings",
        errors,
    );
    expect_u64(
        case,
        &["policy_observation", "declared_max_inspection_age_months"],
        3,
        "maximum inspection age",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "true_safe_suitable_capacity"],
        2,
        "true safe, suitable capacity",
        errors,
    );
    expect_u64(case, &["oracle_facts", "stale_listings"], 1, "stale listing count", errors);
    expect_i64(
        case,
        &["policy_observation", "listing_D3_last_inspected_month"],
        -8,
        "D3 inspection month",
        errors,
    );

    if let Some(dwellings) = value_at(case, &["world", "dwellings"]).and_then(Value::as_array) {
        let true_capacity = dwellings.iter().filter(|dwelling| {
            bool_at(dwelling, &["true_safe"]) == Some(true)
                && bool_at(dwelling, &["true_suitable_for_registered_need"]) == Some(true)
        }).count() as u64;
        if true_capacity != 2 {
            errors.push(format!("derived safe/suitable capacity is {true_capacity}; expected 2"));
        }
        if let Some(fact_capacity) =
            u64_at(case, &["oracle_facts", "true_safe_suitable_capacity"])
        {
            if true_capacity != fact_capacity {
                errors.push("derived capacity does not match oracle_facts".to_owned());
            }
        }
    } else {
        errors.push("world.dwellings must be an array".to_owned());
    }

    if let (Some(last_inspected), Some(max_age)) = (
        i64_at(case, &["policy_observation", "listing_D3_last_inspected_month"]),
        u64_at(case, &["policy_observation", "declared_max_inspection_age_months"]),
    ) {
        let age_at_scenario_start = 0_i64.saturating_sub(last_inspected) as u64;
        if age_at_scenario_start <= max_age {
            errors.push(
                "D3 inspection age must exceed the declared maximum when marked stale".to_owned(),
            );
        }
    }
}

fn validate_h005(case: &Value, errors: &mut Vec<String>) {
    expect_u64(case, &["world", "initial_safe_suitable_units"], 2, "initial capacity", errors);
    expect_u64(
        case,
        &["world", "maintenance_resource_requirement"],
        2,
        "maintenance resources",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "if_maintained", "safe_units_month_1"],
        2,
        "maintained capacity month 1",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "if_maintained", "safe_units_month_2"],
        2,
        "maintained capacity month 2",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "if_maintained", "resource_units_consumed"],
        2,
        "maintenance resources consumed",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "if_skipped", "safe_units_month_1"],
        2,
        "skipped capacity month 1",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "if_skipped", "safe_units_month_2"],
        1,
        "skipped capacity month 2",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "if_skipped", "resource_units_consumed"],
        0,
        "resources consumed when skipped",
        errors,
    );

    if let (Some(maintained), Some(skipped)) = (
        u64_at(case, &["oracle_facts", "if_maintained", "safe_units_month_2"]),
        u64_at(case, &["oracle_facts", "if_skipped", "safe_units_month_2"]),
    ) {
        if maintained <= skipped {
            errors.push(
                "skipping maintenance must expose the encoded later capacity loss".to_owned(),
            );
        }
    }
}

fn validate_h006(case: &Value, errors: &mut Vec<String>) {
    expect_u64(
        case,
        &["world", "cooperative_internal_credit_balance_cu"],
        10,
        "internal credit balance",
        errors,
    );
    expect_u64(
        case,
        &["world", "imported_component_cost_ext"],
        10,
        "external component cost",
        errors,
    );
    expect_u64(
        case,
        &["world", "available_external_settlement_balance_ext"],
        0,
        "external settlement balance",
        errors,
    );
    expect_bool(
        case,
        &["world", "conversion_or_external_settlement_agreement"],
        false,
        "external settlement agreement",
        errors,
    );
    expect_bool(
        case,
        &["oracle_facts", "imported_component_obtainable"],
        false,
        "import feasibility",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "new_capacity_added"],
        0,
        "capacity without imported component",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "external_settlement_shortfall_ext"],
        10,
        "external settlement shortfall",
        errors,
    );
}

fn validate_h007(case: &Value, errors: &mut Vec<String>) {
    expect_u64(case, &["world", "predecessor_loan_balance_cu"], 9, "predecessor liability", errors);
    expect_u64(case, &["world", "new_loan_face_value_cu"], 10, "new loan face value", errors);
    expect_u64(
        case,
        &["world", "proceeds_used_to_repay_predecessor_cu"],
        9,
        "refinancing proceeds",
        errors,
    );
    expect_u64(case, &["world", "proceeds_remaining_cu"], 1, "remaining proceeds", errors);
    expect_u64(
        case,
        &["oracle_facts", "maximum_net_new_financing_from_transaction_cu"],
        1,
        "net new financing",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "physical_capacity_change"],
        0,
        "physical capacity change",
        errors,
    );

    if let (Some(face), Some(repaid), Some(remaining), Some(net_new)) = (
        u64_at(case, &["world", "new_loan_face_value_cu"]),
        u64_at(case, &["world", "proceeds_used_to_repay_predecessor_cu"]),
        u64_at(case, &["world", "proceeds_remaining_cu"]),
        u64_at(case, &["oracle_facts", "maximum_net_new_financing_from_transaction_cu"]),
    ) {
        if face.saturating_sub(repaid) != remaining || remaining != net_new {
            errors.push(
                "new loan must split between predecessor repayment and residual proceeds".to_owned(),
            );
        }
    }
}

fn validate_h008(case: &Value, errors: &mut Vec<String>) {
    expect_u64(case, &["world", "initial_safe_suitable_units"], 1, "initial capacity", errors);
    expect_u64(case, &["world", "tenant_to_owner_payment_cu"], 5, "rent payment", errors);
    expect_bool(
        case,
        &["world", "new_construction_event"],
        false,
        "new construction event",
        errors,
    );
    expect_bool(case, &["world", "improvement_event"], false, "improvement event", errors);
    expect_i64(case, &["oracle_facts", "payer_cash_change_cu"], -5, "payer change", errors);
    expect_i64(case, &["oracle_facts", "recipient_cash_change_cu"], 5, "recipient change", errors);
    expect_u64(
        case,
        &["oracle_facts", "physical_capacity_change"],
        0,
        "physical capacity change",
        errors,
    );
    if let (Some(payer), Some(recipient)) = (
        i64_at(case, &["oracle_facts", "payer_cash_change_cu"]),
        i64_at(case, &["oracle_facts", "recipient_cash_change_cu"]),
    ) {
        if payer + recipient != 0 {
            errors.push("rent payment counterpart entries do not reconcile".to_owned());
        }
    }
}

fn validate_h009(case: &Value, errors: &mut Vec<String>) {
    expect_u64(case, &["world", "total_safe_vacant_units"], 3, "total safe vacant units", errors);
    expect_u64(
        case,
        &["world", "accessible_safe_vacant_units"],
        1,
        "accessible safe units",
        errors,
    );
    expect_u64(
        case,
        &["world", "households_requiring_accessible_units"],
        2,
        "households requiring accessible units",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "maximum_accessible_need_households_housed"],
        1,
        "accessible-need households housed",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "minimum_accessible_need_households_unmet"],
        1,
        "accessible-need households unmet",
        errors,
    );
    expect_bool(
        case,
        &["oracle_facts", "total_units_not_equivalent_to_suitable_capacity"],
        true,
        "suitability distinction",
        errors,
    );

    if let (Some(capacity), Some(housed), Some(unmet), Some(demand)) = (
        u64_at(case, &["world", "accessible_safe_vacant_units"]),
        u64_at(case, &["oracle_facts", "maximum_accessible_need_households_housed"]),
        u64_at(case, &["oracle_facts", "minimum_accessible_need_households_unmet"]),
        u64_at(case, &["world", "households_requiring_accessible_units"]),
    ) {
        if housed > capacity || housed.saturating_add(unmet) != demand {
            errors.push(
                "accessibility allocation must respect capacity and demand closure".to_owned(),
            );
        }
    }
}

fn validate_h010(case: &Value, errors: &mut Vec<String>) {
    expect_u64(
        case,
        &["policy_observation", "reported_confirmation_count"],
        2,
        "reported confirmations",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "unique_source_event_count"],
        1,
        "unique source events",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "independent_confirmation_count"],
        1,
        "independent confirmations",
        errors,
    );
    expect_u64(
        case,
        &["oracle_facts", "true_available_units"],
        2,
        "true available capacity",
        errors,
    );

    let claims = value_at(case, &["policy_observation", "claims"]).and_then(Value::as_array);
    if let Some(claims) = claims {
        let unique_sources = claims
            .iter()
            .filter_map(|claim| str_at(claim, &["source_event_id"]))
            .collect::<BTreeSet<_>>();
        if unique_sources.len() != 1 {
            errors.push(format!(
                "fixture requires one unique source event; derived {}",
                unique_sources.len()
            ));
        }
        if claims.len() != 2 {
            errors.push(format!("fixture requires two mirrored claims; found {}", claims.len()));
        }
        if let Some(expected) = u64_at(case, &["oracle_facts", "unique_source_event_count"]) {
            if unique_sources.len() as u64 != expected {
                errors.push("derived unique source count does not match oracle facts".to_owned());
            }
        }
    } else {
        errors.push("policy_observation.claims must be an array".to_owned());
    }
}

fn make_policy_view(input: &str) -> Result<Value, String> {
    let root: Value = serde_json::from_str(input)
        .map_err(|error| format!("invalid JSON: {error}"))?;
    let cases = root
        .get("cases")
        .and_then(Value::as_array)
        .ok_or("missing cases array")?;
    let mut visible_cases = Vec::<Value>::with_capacity(cases.len());

    for case in cases {
        let case_id = case
            .get("case_id")
            .and_then(Value::as_str)
            .ok_or("case missing case_id")?;
        let purpose = case
            .get("purpose")
            .and_then(Value::as_str)
            .ok_or_else(|| format!("case {case_id} missing purpose"))?;
        let observation = case
            .get("policy_observation")
            .cloned()
            .ok_or_else(|| format!("case {case_id} missing policy_observation"))?;

        if contains_key_recursive(&observation, "world")
            || contains_key_recursive(&observation, "oracle_facts")
            || contains_key_recursive(&observation, "true_safe_suitable_capacity")
        {
            return Err(format!(
                "refusing to expose evaluator-only information in case {case_id}"
            ));
        }

        visible_cases.push(json!({
            "case_id": case_id,
            "purpose": purpose,
            "observation": observation
        }));
    }

    Ok(json!({
        "schema_version": "cecb.housing.policy_view.v0.1",
        "source_schema_version": root["schema_version"],
        "claim_ceiling": concat!(
            "Candidate-visible synthetic observations only; excludes world and oracle facts; ",
            "it is not real-world data."
        ),
        "cases": visible_cases
    }))
}

fn value_at<'a>(root: &'a Value, path: &[&str]) -> Option<&'a Value> {
    path.iter().try_fold(root, |current, key| current.get(*key))
}

fn u64_at(root: &Value, path: &[&str]) -> Option<u64> {
    value_at(root, path).and_then(Value::as_u64)
}

fn i64_at(root: &Value, path: &[&str]) -> Option<i64> {
    value_at(root, path).and_then(Value::as_i64)
}

fn bool_at(root: &Value, path: &[&str]) -> Option<bool> {
    value_at(root, path).and_then(Value::as_bool)
}

fn str_at<'a>(root: &'a Value, path: &[&str]) -> Option<&'a str> {
    value_at(root, path).and_then(Value::as_str)
}

fn expect_u64(
    root: &Value,
    path: &[&str],
    expected: u64,
    label: &str,
    errors: &mut Vec<String>,
) {
    match u64_at(root, path) {
        Some(actual) if actual == expected => {}
        Some(actual) => errors.push(format!("{label}: got {actual}, expected {expected}")),
        None => errors.push(format!("{label}: missing or invalid unsigned integer")),
    }
}

fn expect_i64(
    root: &Value,
    path: &[&str],
    expected: i64,
    label: &str,
    errors: &mut Vec<String>,
) {
    match i64_at(root, path) {
        Some(actual) if actual == expected => {}
        Some(actual) => errors.push(format!("{label}: got {actual}, expected {expected}")),
        None => errors.push(format!("{label}: missing or invalid signed integer")),
    }
}

fn expect_bool(
    root: &Value,
    path: &[&str],
    expected: bool,
    label: &str,
    errors: &mut Vec<String>,
) {
    match bool_at(root, path) {
        Some(actual) if actual == expected => {}
        Some(actual) => errors.push(format!("{label}: got {actual}, expected {expected}")),
        None => errors.push(format!("{label}: missing or invalid boolean")),
    }
}

fn contains_key_recursive(value: &Value, target: &str) -> bool {
    match value {
        Value::Object(object) => {
            object.contains_key(target)
                || object.values().any(|child| contains_key_recursive(child, target))
        }
        Value::Array(array) => array
            .iter()
            .any(|child| contains_key_recursive(child, target)),
        _ => false,
    }
}

fn collect_object_keys(value: &Value, keys: &mut BTreeSet<String>) {
    match value {
        Value::Object(object) => {
            for (key, child) in object {
                keys.insert(key.clone());
                collect_object_keys(child, keys);
            }
        }
        Value::Array(array) => {
            for child in array {
                collect_object_keys(child, keys);
            }
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture_value() -> Value {
        serde_json::from_str(EMBEDDED_FIXTURE).expect("embedded fixture must be valid JSON")
    }

    fn rejects_mutation(mut fixture: Value, mutate: impl FnOnce(&mut Value)) {
        mutate(&mut fixture);
        let input = serde_json::to_string(&fixture).expect("mutated fixture serializes");
        let (_, passed) = validate_fixture(&input).expect("mutated fixture parses");
        assert!(!passed, "known-invalid fixture mutation unexpectedly passed");
    }

    #[test]
    fn canonical_fixture_passes_all_ten_known_truth_cases() {
        let (report, passed) = validate_fixture(EMBEDDED_FIXTURE).expect("fixture parses");
        assert!(passed, "{report:#}");
        assert_eq!(report["case_count"], 10);
        assert_eq!(report["case_pass_count"], 10);
        assert_eq!(report["case_fail_count"], 0);
    }

    #[test]
    fn policy_view_excludes_world_and_oracle_truth() {
        let view = make_policy_view(EMBEDDED_FIXTURE).expect("policy view builds");
        let serialized = serde_json::to_string(&view).expect("policy view serializes");
        assert!(!serialized.contains("oracle_facts"));
        assert!(!serialized.contains("true_safe_suitable_capacity"));
        assert!(!serialized.contains("\"world\""));
        assert_eq!(view["cases"].as_array().map(Vec::len), Some(10));
    }

    #[test]
    fn rejects_capacity_claim_above_physical_stock() {
        rejects_mutation(fixture_value(), |fixture| {
            fixture["cases"][0]["oracle_facts"]["maximum_households_housed_by_end"] = json!(3);
        });
    }

    #[test]
    fn rejects_housing_capacity_before_construction_completion() {
        rejects_mutation(fixture_value(), |fixture| {
            fixture["cases"][1]["oracle_facts"]["safe_suitable_units_before_completion"] = json!(3);
        });
    }

    #[test]
    fn rejects_grant_that_does_not_reconcile_or_uses_no_real_resources() {
        rejects_mutation(fixture_value(), |fixture| {
            let facts = &mut fixture["cases"][2]["oracle_facts"];
            facts["physical_resource_consumed_if_completed"] = json!(0);
        });
    }

    #[test]
    fn rejects_listing_claimed_stale_when_inspection_age_is_current() {
        rejects_mutation(fixture_value(), |fixture| {
            let observation = &mut fixture["cases"][3]["policy_observation"];
            observation["listing_D3_last_inspected_month"] = json!(-1);
        });
    }

    #[test]
    fn rejects_deferred_maintenance_without_the_encoded_capacity_loss() {
        rejects_mutation(fixture_value(), |fixture| {
            fixture["cases"][4]["oracle_facts"]["if_skipped"]["safe_units_month_2"] = json!(2);
        });
    }

    #[test]
    fn rejects_internal_credit_as_external_settlement_without_a_bridge() {
        rejects_mutation(fixture_value(), |fixture| {
            fixture["cases"][5]["oracle_facts"]["imported_component_obtainable"] = json!(true);
        });
    }

    #[test]
    fn rejects_refinancing_face_value_as_net_new_capital() {
        rejects_mutation(fixture_value(), |fixture| {
            let facts = &mut fixture["cases"][6]["oracle_facts"];
            facts["maximum_net_new_financing_from_transaction_cu"] = json!(10);
        });
    }

    #[test]
    fn rejects_rent_payment_as_new_physical_capacity() {
        rejects_mutation(fixture_value(), |fixture| {
            fixture["cases"][7]["oracle_facts"]["physical_capacity_change"] = json!(1);
        });
    }

    #[test]
    fn rejects_nominal_unit_count_as_accessible_capacity() {
        rejects_mutation(fixture_value(), |fixture| {
            let facts = &mut fixture["cases"][8]["oracle_facts"];
            facts["maximum_accessible_need_households_housed"] = json!(2);
        });
    }

    #[test]
    fn rejects_mirrored_records_as_independent_confirmations() {
        rejects_mutation(fixture_value(), |fixture| {
            fixture["cases"][9]["oracle_facts"]["independent_confirmation_count"] = json!(2);
        });
    }

    #[test]
    fn rejects_duplicate_case_ids() {
        rejects_mutation(fixture_value(), |fixture| {
            fixture["cases"][1]["case_id"] = json!("H001_FIXED_CAPACITY_CEILING");
        });
    }

    #[test]
    fn rejects_oracle_leak_into_candidate_observation() {
        rejects_mutation(fixture_value(), |fixture| {
            fixture["cases"][0]["policy_observation"]["true_safe_suitable_capacity"] = json!(2);
        });
    }

    #[test]
    fn rejects_future_oracle_fields_leaking_without_a_denylist_update() {
        rejects_mutation(fixture_value(), |fixture| {
            fixture["cases"][9]["policy_observation"]["unique_source_event_count"] = json!(1);
        });
    }

    #[test]
    fn rejects_unregistered_fixture_schema_version() {
        let mut fixture = fixture_value();
        fixture["schema_version"] = json!("cecb.housing.known_truth.v0.2");
        let input = serde_json::to_string(&fixture).expect("fixture serializes");
        assert!(validate_fixture(&input).is_err());
    }

    #[test]
    fn rejects_missing_case_from_frozen_v0_1_corpus() {
        rejects_mutation(fixture_value(), |fixture| {
            fixture["cases"].as_array_mut().expect("cases array").remove(9);
        });
    }
}
