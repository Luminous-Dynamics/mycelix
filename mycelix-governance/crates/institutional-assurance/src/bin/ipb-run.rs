use institutional_assurance::conflict_of_interest_suite;

fn main() {
    let report = conflict_of_interest_suite();
    println!(
        "{}",
        serde_json::to_string_pretty(&report)
            .expect("benchmark report serializes to JSON")
    );

    if report.failed_cases != 0 {
        std::process::exit(1);
    }
}
