use measurement_evidence_invalidation::validate_tracked;
use std::process::ExitCode;

fn main() -> ExitCode {
    match validate_tracked() {
        Ok(case_count) => {
            println!("suite=measurement_evidence_invalidation_v0_1");
            println!("cases={case_count}");
            println!("result=all_known_answers_matched");
            println!("claim_ceiling=deterministic_dependency_and_invalidation_semantics_only");
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("result=invalid");
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}
