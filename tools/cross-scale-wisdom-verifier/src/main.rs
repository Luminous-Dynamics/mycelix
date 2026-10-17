use std::{env, process};

use cross_scale_wisdom_verifier::verify_files;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 5 || args[1] != "verify" {
        eprintln!(
            "usage: cross-scale-wisdom-verifier verify <corpus.jsonl> <manifest.json> <case-schema.json>"
        );
        process::exit(2);
    }

    match verify_files(&args[2], &args[3], &args[4]) {
        Ok(report) => {
            match serde_json::to_string_pretty(&report) {
                Ok(output) => println!("{output}"),
                Err(error) => {
                    eprintln!("failed to serialize verification report: {error}");
                    process::exit(2);
                }
            }
        }
        Err(error) => {
            eprintln!("corpus verification failed closed: {error}");
            process::exit(2);
        }
    }
}
