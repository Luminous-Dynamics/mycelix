use relationship_consent_model::bounded_model_check;

fn parse_arg(index: usize, default: usize) -> usize {
    std::env::args()
        .nth(index)
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(default)
}

fn main() {
    let max_depth = parse_arg(1, 6);
    let max_states = parse_arg(2, 100_000);
    let report = bounded_model_check(max_depth, max_states);

    println!(
        "{{\"max_depth\":{},\"unique_states\":{},\"transitions_checked\":{},\"maximum_depth_explored\":{},\"state_cap_reached\":{},\"invariant_violation\":{}}}",
        max_depth,
        report.unique_states,
        report.transitions_checked,
        report.maximum_depth_explored,
        report.state_cap_reached,
        report.invariant_violation
    );

    if report.state_cap_reached || report.invariant_violation || report.maximum_depth_explored < max_depth {
        std::process::exit(2);
    }
}
