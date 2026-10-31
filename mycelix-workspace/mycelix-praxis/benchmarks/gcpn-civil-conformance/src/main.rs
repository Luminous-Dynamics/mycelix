use gcpn_civil_conformance::fixture_report;

fn main() {
    match fixture_report() {
        Ok(report) => {
            println!("GCPN synthetic civilian-protection conformance v0.1");
            println!("cases: {}", report.cases);
            println!("integrity_conformance: {}", if report.conformance_passes() { "PASS" } else { "FAIL" });
            println!("independence_promotions: {}", report.independence_promotions);
            println!("measurement_state_coercions: {}", report.measurement_state_coercions);
            println!("authenticity_measurement_conflations: {}", report.authenticity_measurement_conflations);
            println!("unauthorized_alerts: {}", report.unauthorized_alerts);
            println!("local_partition_continuity: {}/{}", report.local_partition_deliveries, report.local_partition_cases);
            println!("untraceable_alerts: {}", report.untraceable_alerts);
            println!("unauthorized_remote_mutations: {}", report.unauthorized_remote_mutations);
            println!("receiver_delivery: {:.4} ({}/{})", report.receiver_delivery_ratio(), report.receipts, report.recipients);
            println!("receiver_accessibility_coverage: {:.4} ({}/{})", report.accessibility_coverage_ratio(), report.accessible_receivers, report.recipients);
            if !report.conformance_passes() {
                std::process::exit(1);
            }
        }
        Err(error) => {
            eprintln!("fixture error: {error}");
            std::process::exit(2);
        }
    }
}
