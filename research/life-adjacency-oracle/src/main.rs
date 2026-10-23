use std::fmt::Write as _;

const RECORD_COUNTS_PER_PERSON: [usize; 2] = [1, 3];
const GROUP_SIZES: [usize; 3] = [1, 2, 4];
const MAX_EXACT_ENUMERATION_FACTS: usize = 16;
const PPM: u64 = 1_000_000;
const MANIFEST: &str = include_str!("../manifest.json");

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mechanism {
    RandomizedResponseAllChangedFacts,
    RandomizedRecordsPublishRelationEdgesRaw,
}

impl Mechanism {
    const ALL: [Self; 2] = [
        Self::RandomizedResponseAllChangedFacts,
        Self::RandomizedRecordsPublishRelationEdgesRaw,
    ];

    const fn name(self) -> &'static str {
        match self {
            Self::RandomizedResponseAllChangedFacts => "randomized_response_all_changed_facts",
            Self::RandomizedRecordsPublishRelationEdgesRaw => {
                "randomized_records_publish_relation_edges_raw"
            }
        }
    }

    const fn publishes_edges_in_clear(self) -> bool {
        matches!(self, Self::RandomizedRecordsPublishRelationEdgesRaw)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct AdjacencyCase {
    name: &'static str,
    group_size: usize,
    records_per_person: usize,
    changed_record_facts: usize,
    changed_relation_edges: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RatioEnumeration {
    states_examined: u64,
    max_log3_ratio_exponent: i64,
    min_log3_ratio_exponent: i64,
    exact: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct ReportRow {
    adjacency: &'static str,
    mechanism: &'static str,
    group_size: usize,
    records_per_person: usize,
    changed_record_facts: usize,
    changed_relation_edges: usize,
    privatized_fact_count: usize,
    enumerated_output_states: u64,
    privatized_channel_max_log3_ratio_exponent: i64,
    epsilon_nats: String,
    finite_epsilon: bool,
    delta_lower_bound_ppm: u64,
    relation_output_support_disjoint: bool,
    exact_enumeration_passed: bool,
}

fn frozen_cases() -> Vec<AdjacencyCase> {
    let mut cases = vec![AdjacencyCase {
        name: "record_adjacency",
        group_size: 1,
        records_per_person: 1,
        changed_record_facts: 1,
        changed_relation_edges: 0,
    }];

    for records_per_person in RECORD_COUNTS_PER_PERSON {
        cases.push(AdjacencyCase {
            name: "person_adjacency",
            group_size: 1,
            records_per_person,
            changed_record_facts: records_per_person,
            changed_relation_edges: 0,
        });
    }

    for group_size in GROUP_SIZES {
        for records_per_person in RECORD_COUNTS_PER_PERSON {
            cases.push(AdjacencyCase {
                name: "group_adjacency",
                group_size,
                records_per_person,
                changed_record_facts: group_size * records_per_person,
                changed_relation_edges: 0,
            });
        }
    }

    for group_size in GROUP_SIZES {
        for records_per_person in RECORD_COUNTS_PER_PERSON {
            cases.push(AdjacencyCase {
                name: "relational_graph_adjacency",
                group_size,
                records_per_person,
                changed_record_facts: group_size * records_per_person,
                // One protected relationship edge changes per member.
                changed_relation_edges: group_size,
            });
        }
    }

    cases
}

// For one binary randomized-response fact, truthful output has probability 3/4
// and the opposite output has probability 1/4. Given two adjacent datasets
// that differ on m bits, for every output mask the exact likelihood ratio is
// 3^(zero_bits - one_bits). Enumerating all 2^m masks verifies the maximum
// exponent m and minimum exponent -m without floating-point probability math.
fn enumerate_binary_rr_ratio(changed_facts: usize) -> RatioEnumeration {
    assert!(changed_facts <= MAX_EXACT_ENUMERATION_FACTS);
    let states = 1_u64 << changed_facts;
    let mut maximum = i64::MIN;
    let mut minimum = i64::MAX;

    for mask in 0..states {
        let ones = i64::from(mask.count_ones());
        let exponent = changed_facts as i64 - 2 * ones;
        maximum = maximum.max(exponent);
        minimum = minimum.min(exponent);
    }

    RatioEnumeration {
        states_examined: states,
        max_log3_ratio_exponent: maximum,
        min_log3_ratio_exponent: minimum,
        exact: maximum == changed_facts as i64 && minimum == -(changed_facts as i64),
    }
}

fn evaluate_case(case: AdjacencyCase, mechanism: Mechanism) -> ReportRow {
    let privatized_fact_count = match mechanism {
        Mechanism::RandomizedResponseAllChangedFacts => {
            case.changed_record_facts + case.changed_relation_edges
        }
        Mechanism::RandomizedRecordsPublishRelationEdgesRaw => case.changed_record_facts,
    };
    let ratio = enumerate_binary_rr_ratio(privatized_fact_count);
    let relation_output_support_disjoint =
        mechanism.publishes_edges_in_clear() && case.changed_relation_edges > 0;
    let finite_epsilon = !relation_output_support_disjoint;
    let delta_lower_bound_ppm = if relation_output_support_disjoint {
        PPM
    } else {
        0
    };
    let epsilon_nats = if finite_epsilon {
        let base_fact_epsilon = 3.0_f64.ln();
        format!(
            "{:.9}",
            ratio.max_log3_ratio_exponent as f64 * base_fact_epsilon
        )
    } else {
        "INF".to_owned()
    };

    ReportRow {
        adjacency: case.name,
        mechanism: mechanism.name(),
        group_size: case.group_size,
        records_per_person: case.records_per_person,
        changed_record_facts: case.changed_record_facts,
        changed_relation_edges: case.changed_relation_edges,
        privatized_fact_count,
        enumerated_output_states: ratio.states_examined,
        privatized_channel_max_log3_ratio_exponent: ratio.max_log3_ratio_exponent,
        epsilon_nats,
        finite_epsilon,
        delta_lower_bound_ppm,
        relation_output_support_disjoint,
        exact_enumeration_passed: ratio.exact,
    }
}

fn render_report() -> String {
    debug_assert!(!MANIFEST.is_empty());
    let mut output = String::new();
    writeln!(
        &mut output,
        "adjacency,mechanism,group_size,records_per_person,changed_record_facts,changed_relation_edges,privatized_fact_count,enumerated_output_states,privatized_channel_max_log3_ratio_exponent,epsilon_nats,finite_epsilon,delta_lower_bound_ppm,relation_output_support_disjoint,exact_enumeration_passed"
    )
    .expect("writing to String cannot fail");

    for case in frozen_cases() {
        for mechanism in Mechanism::ALL {
            let row = evaluate_case(case, mechanism);
            writeln!(
                &mut output,
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                row.adjacency,
                row.mechanism,
                row.group_size,
                row.records_per_person,
                row.changed_record_facts,
                row.changed_relation_edges,
                row.privatized_fact_count,
                row.enumerated_output_states,
                row.privatized_channel_max_log3_ratio_exponent,
                row.epsilon_nats,
                row.finite_epsilon,
                row.delta_lower_bound_ppm,
                row.relation_output_support_disjoint,
                row.exact_enumeration_passed,
            )
            .expect("writing to String cannot fail");
        }
    }

    output
}

fn main() {
    print!("{}", render_report());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn manifest_freezes_units_mechanisms_and_exact_enumeration_limit() {
        for adjacency in ["record", "person", "group", "relational_graph"] {
            assert!(
                MANIFEST.contains(adjacency),
                "missing adjacency unit {adjacency}"
            );
        }
        for mechanism in Mechanism::ALL {
            assert!(
                MANIFEST.contains(mechanism.name()),
                "missing {}",
                mechanism.name()
            );
        }
        assert!(MANIFEST.contains("\"true_value_release_probability\": \"3/4\""));
        assert!(MANIFEST.contains("\"base_epsilon\": \"ln(3)\""));
        assert!(MANIFEST.contains("\"max_changed_facts\": 16"));
    }

    #[test]
    fn exact_enumeration_checks_every_binary_output_vector_through_sixteen_facts() {
        for changed_facts in 1..=MAX_EXACT_ENUMERATION_FACTS {
            let result = enumerate_binary_rr_ratio(changed_facts);
            assert!(result.exact);
            assert_eq!(result.states_examined, 1_u64 << changed_facts);
            assert_eq!(result.max_log3_ratio_exponent, changed_facts as i64);
            assert_eq!(result.min_log3_ratio_exponent, -(changed_facts as i64));
        }
    }

    #[test]
    fn epsilon_scales_with_the_declared_privacy_unit() {
        let cases = frozen_cases();
        let record = cases
            .iter()
            .find(|case| case.name == "record_adjacency")
            .expect("record case");
        let person_three_records = cases
            .iter()
            .find(|case| case.name == "person_adjacency" && case.records_per_person == 3)
            .expect("person case");
        let group_four_by_three = cases
            .iter()
            .find(|case| {
                case.name == "group_adjacency"
                    && case.group_size == 4
                    && case.records_per_person == 3
            })
            .expect("group case");
        let graph_four_by_three = cases
            .iter()
            .find(|case| {
                case.name == "relational_graph_adjacency"
                    && case.group_size == 4
                    && case.records_per_person == 3
            })
            .expect("graph case");

        let record_row = evaluate_case(*record, Mechanism::RandomizedResponseAllChangedFacts);
        let person_row = evaluate_case(
            *person_three_records,
            Mechanism::RandomizedResponseAllChangedFacts,
        );
        let group_row = evaluate_case(
            *group_four_by_three,
            Mechanism::RandomizedResponseAllChangedFacts,
        );
        let graph_row = evaluate_case(
            *graph_four_by_three,
            Mechanism::RandomizedResponseAllChangedFacts,
        );

        assert_eq!(record_row.privatized_channel_max_log3_ratio_exponent, 1);
        assert_eq!(person_row.privatized_channel_max_log3_ratio_exponent, 3);
        assert_eq!(group_row.privatized_channel_max_log3_ratio_exponent, 12);
        assert_eq!(graph_row.privatized_channel_max_log3_ratio_exponent, 16);
        assert!(record_row.finite_epsilon && person_row.finite_epsilon);
        assert!(group_row.finite_epsilon && graph_row.finite_epsilon);
        assert_eq!(graph_row.privatized_fact_count, 16);
        assert_eq!(graph_row.enumerated_output_states, 65_536);
    }

    #[test]
    fn clear_relation_edge_breaks_finite_epsilon_for_graph_adjacency() {
        let graph = frozen_cases()
            .into_iter()
            .find(|case| {
                case.name == "relational_graph_adjacency"
                    && case.group_size == 4
                    && case.records_per_person == 3
            })
            .expect("graph case");
        let leaky = evaluate_case(graph, Mechanism::RandomizedRecordsPublishRelationEdgesRaw);
        assert!(!leaky.finite_epsilon);
        assert_eq!(leaky.epsilon_nats, "INF");
        assert!(leaky.relation_output_support_disjoint);
        assert_eq!(leaky.delta_lower_bound_ppm, PPM);
        // The enumerated channel still correctly accounts for the randomized
        // records, but the clear edge makes the full release fail finite epsilon.
        assert_eq!(leaky.privatized_fact_count, 12);
        assert_eq!(leaky.privatized_channel_max_log3_ratio_exponent, 12);
    }

    #[test]
    fn raw_edge_does_not_change_record_adjacent_epsilon_when_edge_is_unchanged() {
        let record = frozen_cases()
            .into_iter()
            .find(|case| case.name == "record_adjacency")
            .expect("record case");
        let row = evaluate_case(record, Mechanism::RandomizedRecordsPublishRelationEdgesRaw);
        assert!(row.finite_epsilon);
        assert!(!row.relation_output_support_disjoint);
        assert_eq!(row.privatized_channel_max_log3_ratio_exponent, 1);
        assert_eq!(row.delta_lower_bound_ppm, 0);
    }

    #[test]
    fn report_is_deterministic_rectangular_and_explicit_about_unbounded_rows() {
        let first = render_report();
        let second = render_report();
        assert_eq!(first, second);
        let mut lines = first.lines();
        let header = lines.next().expect("header exists");
        let width = header.split(',').count();
        assert_eq!(width, 14);
        for field in [
            "changed_record_facts",
            "changed_relation_edges",
            "privatized_fact_count",
            "epsilon_nats",
            "finite_epsilon",
            "delta_lower_bound_ppm",
            "relation_output_support_disjoint",
        ] {
            assert!(header.contains(field));
        }
        let rows = lines.collect::<Vec<_>>();
        assert_eq!(rows.len(), frozen_cases().len() * Mechanism::ALL.len());
        assert!(
            rows.iter()
                .any(|row| row.contains(",INF,false,1000000,true,"))
        );
        for row in rows {
            assert_eq!(row.split(',').count(), width, "bad row: {row}");
        }
    }
}
