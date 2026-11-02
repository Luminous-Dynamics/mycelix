use gcpn_e2e_fault_matrix::run_full_matrix;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let report = run_full_matrix()?;

    println!("# Synthetic GCPN evidence-chain fault matrix.");
    println!("# Six-stage assurance and civil evidence contracts are separately versioned adapters.");
    println!("# No operational warning authority or live transport is invoked.");
    print!("{}", report.to_csv());

    println!(
        "# coverage: original_fault_families={} all_singles_detected={} six_stage_pairs={}/{} civil_pairs={}/{} civil_fail_closed={}",
        report.original_fault_families,
        report.all_single_faults_detected(),
        report.six_stage_pairwise_no_false_support,
        report.six_stage_pairwise_cases,
        report.civil_pairwise_detected,
        report.civil_pairwise_cases,
        report.civil_pairwise_fail_closed
    );

    if !report.all_single_faults_detected() || !report.all_pairwise_faults_detected() {
        return Err("integrated fault matrix contains an undetected or false-support interaction".into());
    }

    Ok(())
}
