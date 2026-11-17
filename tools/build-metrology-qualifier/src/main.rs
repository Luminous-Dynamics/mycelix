use build_metrology_qualifier::validate_tracked;
use std::process::ExitCode;

fn main() -> ExitCode {
    match validate_tracked() {
        Ok(count) => {
            println!("suite=building_metrology_synthetic_admission_v0_1");
            println!("cases={count}");
            println!("result=all_known_answers_matched");
            println!("claim_ceiling=synthetic_profile_and_admission_logic_only");
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("result=invalid");
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}
