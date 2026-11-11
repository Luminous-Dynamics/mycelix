use build_derived_measurement_graph::validate_tracked;
use std::process::ExitCode;

fn main() -> ExitCode {
    match validate_tracked() {
        Ok(count) => {
            println!("suite=build_derived_measurement_graph_v0_1");
            println!("cases={count}");
            println!("result=all_known_answers_matched");
            println!("claim_ceiling=synthetic_graph_validation_only");
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("result=invalid");
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}
