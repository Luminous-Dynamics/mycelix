use building_measurement_conditions::validate_tracked;
use std::process::ExitCode;

fn main() -> ExitCode {
    match validate_tracked() {
        Ok((profiles, cases)) => {
            println!("suite=building_measurement_conditions_v0_1");
            println!("profiles={profiles}");
            println!("cases={cases}");
            println!("result=all_known_answers_matched");
            println!("claim_ceiling=synthetic_condition_applicability_only");
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("result=invalid");
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}
