use gcpn_forecast_coverage_qualification::{
    analytic_truth, basic_bootstrap_interval, bootstrap_interval, generate_dataset,
    studentized_system_bootstrap_interval, BootstrapUnit, DgpConfig,
};

fn main() {
    let config = DgpConfig::default();
    let seed = 0x6a09_e667_f3bc_c909;
    let truth = match analytic_truth(&config) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("invalid DGP: {error}");
            std::process::exit(2);
        }
    };
    let dataset = match generate_dataset(&config, seed) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("dataset generation failed: {error}");
            std::process::exit(2);
        }
    };

    println!("GCPN known-DGP forecast interval qualification v0.1");
    println!("status: synthetic_exploratory_method_demonstration");
    println!("seed: 0x{seed:016x}");
    println!("independent_systems: {}", dataset.systems.len());
    println!("nested_event_ids: {}", dataset.event_count());
    println!("matched_forecast_cases: {}", dataset.case_count());
    println!("expected_cases_per_system: {:.6}", truth.expected_cases_per_system);
    println!("oracle_case_weighted_difference: {:.8}", truth.case_weighted);
    println!("oracle_system_weighted_difference: {:.8}", truth.system_weighted);
    println!("low_state_expected_difference: {:.8}", truth.low_state_expected_difference);
    println!("high_state_expected_difference: {:.8}", truth.high_state_expected_difference);
    println!("interpretation: case- and system-weighted estimands differ; they are not interchangeable");
    println!();
    println!("method,resampling_units,estimate,lower_95,upper_95,bootstrap_replicates");
    for method in BootstrapUnit::ALL {
        match bootstrap_interval(&dataset, method, 0.95, 20_000, seed ^ 0x510e_527f_ade6_82d1) {
            Ok(interval) => println!(
                "{},{},{:.8},{:.8},{:.8},{}",
                method.label(),
                interval.resampling_units,
                interval.estimate,
                interval.lower,
                interval.upper,
                interval.replicates
            ),
            Err(error) => {
                eprintln!("{} failed: {error}", method.label());
                std::process::exit(1);
            }
        }
    }
    match basic_bootstrap_interval(&dataset, BootstrapUnit::IndependentSystem, 0.95, 20_000, seed ^ 0x1f83_d9ab_fb41_bd6b) {
        Ok(interval) => println!(
            "independent_system_basic,{},{:.8},{:.8},{:.8},{}",
            interval.resampling_units, interval.estimate, interval.lower, interval.upper, interval.replicates
        ),
        Err(error) => { eprintln!("basic interval failed: {error}"); std::process::exit(1); }
    }
    match studentized_system_bootstrap_interval(&dataset, 0.95, 20_000, seed ^ 0x5be0_cd19_137e_2179) {
        Ok(interval) => println!(
            "independent_system_studentized_t,{},{:.8},{:.8},{:.8},{}",
            interval.independent_systems, interval.estimate, interval.lower, interval.upper, interval.valid_replicates
        ),
        Err(error) => { eprintln!("studentized interval failed: {error}"); std::process::exit(1); }
    }
    println!();
    println!("non_claim: no empirical coverage qualification or real forecast comparison has been performed");
    println!("warning: IID-case and event-level methods are negative controls when systems share latent state");
}
