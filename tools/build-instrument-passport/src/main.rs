use building_instrument_passport::validate_tracked;
use std::process::ExitCode;

fn main() -> ExitCode {
    match validate_tracked() {
        Ok(case_count) => {
            println!("suite=building_instrument_passport_v0_1");
            println!("cases={case_count}");
            println!("result=all_known_answers_matched");
            println!("claim_ceiling=synthetic_instrument_passport_applicability_only");
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("result=invalid");
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}
