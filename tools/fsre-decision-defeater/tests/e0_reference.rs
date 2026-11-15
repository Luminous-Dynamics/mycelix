use fsre_decision_defeater::analyze;

const FIXTURE: &str = include_str!("../examples/pump-station.psv");

/// Independent fixture oracle: this deliberately does not call the crate's
/// decision-signature or mutation-application functions. It directly evaluates
/// the two repair safety cells and the known mutation costs.
#[test]
fn exhaustive_subset_oracle_agrees_on_minimum_repair_defeaters() {
    let mutation_ids = ["m-a", "m-b", "m-noop"];
    let mutation_costs = [2_u128, 2_u128, 1_u128];

    let mut expected_minimum: Option<u128> = None;
    let mut expected_masks = Vec::new();

    for mask in 0_u8..8 {
        let repair_w1_safety = 80_i64 - if mask & 0b001 != 0 { 30 } else { 0 };
        let repair_w2_safety = 80_i64 - if mask & 0b010 != 0 { 30 } else { 0 };
        let repair_remains_robustly_safe =
            repair_w1_safety >= 60 && repair_w2_safety >= 60;

        if repair_remains_robustly_safe {
            continue;
        }

        let mut cost = 0_u128;
        for (index, mutation_cost) in mutation_costs.iter().enumerate() {
            if mask & (1_u8 << index) != 0 {
                cost += *mutation_cost;
            }
        }

        match expected_minimum {
            None => {
                expected_minimum = Some(cost);
                expected_masks.clear();
                expected_masks.push(mask);
            }
            Some(current) if cost < current => {
                expected_minimum = Some(cost);
                expected_masks.clear();
                expected_masks.push(mask);
            }
            Some(current) if cost == current => expected_masks.push(mask),
            Some(_) => {}
        }
    }

    let expected_ids: Vec<Vec<String>> = expected_masks
        .iter()
        .map(|mask| {
            mutation_ids
                .iter()
                .enumerate()
                .filter(|&(index, _)| *mask & (1_u8 << index) != 0)
                .map(|(_, id)| (*id).to_owned())
                .collect()
        })
        .collect();

    let report = analyze(FIXTURE).expect("fixture parses").expect("baseline qualifies");
    assert_eq!(expected_minimum, Some(2));
    assert_eq!(report.robustness_margin, expected_minimum);
    assert_eq!(report.status, "DEFEATER_FOUND");
    assert_eq!(report.minimality_status, "PROVEN_MINIMAL");

    let actual_ids: Vec<Vec<String>> = report
        .witnesses
        .iter()
        .map(|witness| witness.mutation_ids.clone())
        .collect();
    assert_eq!(actual_ids, expected_ids);
    assert_eq!(report.witness_count as usize, expected_ids.len());
}
