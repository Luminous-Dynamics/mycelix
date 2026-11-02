use gcpn_e2e_fault_interaction::{BenchmarkReport, standard_catalogue};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let scenarios = standard_catalogue()?;
    let report = BenchmarkReport::from_scenarios(&scenarios);

    println!("# Synthetic oracle only; no component adapter has been executed.");
    println!("{}", report.summary_line());
    print!("{}", report.to_csv());

    Ok(())
}
