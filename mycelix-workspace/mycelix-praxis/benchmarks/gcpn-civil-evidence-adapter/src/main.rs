use gcpn_civil_evidence_adapter::{execute_faults, Fault};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("# Synthetic adapter against the GCPN civil-evidence contract auditor.");
    println!("# This is not a live warning or recipient service.");
    println!("fault,result,authority,delivery,accessibility,comprehension,action,case_frame,detail");

    for fault in Fault::ALL {
        match execute_faults(&[fault]) {
            Ok(report) => {
                let delivery = report
                    .delivery
                    .first()
                    .map(|item| label(item.disposition))
                    .unwrap_or("missing");
                let accessibility = report
                    .accessibility
                    .first()
                    .map(|item| label(item.disposition))
                    .unwrap_or("missing");
                let comprehension = report
                    .comprehension
                    .first()
                    .map(|item| label(item.disposition))
                    .unwrap_or("missing");
                let action = report
                    .actions
                    .first()
                    .map(|item| label(item.disposition))
                    .unwrap_or("missing");

                println!(
                    "{},executed,{},{},{},{},{},{},{}",
                    fault.label(),
                    label(report.authority.disposition),
                    delivery,
                    accessibility,
                    comprehension,
                    action,
                    label(report.case_frame.disposition),
                    "actual_compositional_civil_auditor"
                );
            }
            Err(error) => {
                println!(
                    "{},fail_closed,,,,,,,{error}",
                    fault.label()
                );
            }
        }
    }

    let report = execute_faults(&[])?;
    println!("# Clean synthetic baseline:");
    print!("{}", report.to_csv());
    Ok(())
}

fn label(value: gcpn_compositional_assurance::ClaimDisposition) -> &'static str {
    match value {
        gcpn_compositional_assurance::ClaimDisposition::Supports => "supports",
        gcpn_compositional_assurance::ClaimDisposition::Refutes => "refutes",
        gcpn_compositional_assurance::ClaimDisposition::Indeterminate => "indeterminate",
        gcpn_compositional_assurance::ClaimDisposition::OutOfScope => "out_of_scope",
        gcpn_compositional_assurance::ClaimDisposition::Unavailable => "unavailable",
    }
}
