use ingress_receipt_qualifier::validate_tracked_corpus;
use std::process::ExitCode;

fn main() -> ExitCode {
    match validate_tracked_corpus() {
        Ok(count) => {
            println!("suite=ingress_receipt_qualification_v0_1");
            println!("cases={count}");
            println!("result=all_known_answers_matched");
            println!("claim_ceiling=synthetic_interval_and_owner_receipt_reference_semantics_only");
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("result=invalid");
            eprintln!("{error:?}");
            ExitCode::FAILURE
        }
    }
}
