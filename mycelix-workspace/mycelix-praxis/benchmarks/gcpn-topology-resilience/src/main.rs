use gcpn_topology_resilience::{fixture_comparison, Architecture};

fn main() {
    match fixture_comparison() {
        Ok(rows) => {
            println!("GCPN deterministic civilian topology comparison v0.1");
            println!("scenario,architecture,intended,delivered,missed,local_delivered,remote_delivered,delivery_ratio");
            for row in rows {
                let o = row.outcome;
                println!(
                    "{},{},{},{},{},{},{},{:.4}",
                    row.scenario_id,
                    row.architecture.label(),
                    o.intended_receivers,
                    o.delivered(),
                    o.missed(),
                    o.local_delivered,
                    o.remote_delivered(),
                    o.delivery_ratio()
                );
            }
            println!("architectures: {}", Architecture::ALL.len());
            println!("interpretation: deterministic structural arithmetic only; not observed performance");
        }
        Err(error) => {
            eprintln!("fixture error: {error}");
            std::process::exit(2);
        }
    }
}
