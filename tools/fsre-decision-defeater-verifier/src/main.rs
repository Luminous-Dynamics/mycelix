use fsre_decision_defeater_verifier::verify_package;
use std::{env, process::ExitCode};

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 || args[1] != "verify" {
        eprintln!("usage: fsre-decision-defeater-verifier verify <evidence-package-dir>");
        return ExitCode::from(2);
    }
    match verify_package(&args[2]) {
        Ok(receipt) => {
            print!("{}", receipt.render());
            ExitCode::SUCCESS
        }
        Err(error) => {
            println!("disposition={}", error.disposition.as_str());
            println!("detail={}", error.detail.replace('\n', " "));
            ExitCode::from(3)
        }
    }
}
