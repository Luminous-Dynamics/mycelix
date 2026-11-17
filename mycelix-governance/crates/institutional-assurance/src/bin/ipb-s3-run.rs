use institutional_assurance::capture::adversarial_capture_suite;

fn main() {
    let report = adversarial_capture_suite();
    println!(
        "{}",
        serde_json::to_string_pretty(&report).expect("S3 benchmark report serializes to JSON")
    );

    if report.failed_cases != 0 {
        std::process::exit(1);
    }
}
