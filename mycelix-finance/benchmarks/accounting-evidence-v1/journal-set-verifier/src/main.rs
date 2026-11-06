use fin_journal_set_verifier::verify;
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

fn main() {
    if let Err(error) = run() {
        eprintln!("fin-journal-set-verifier: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut candidate = PathBuf::from("../fin_journal_set_candidates_v1.1.0.json");
    let mut candidate_schema = PathBuf::from("../FIN_JOURNAL_SET_INPUT_V1.1.0.schema.json");
    let mut oracle =
        PathBuf::from("../public-journal-set-oracle/fin_journal_set_oracle_v1.1.0.json");
    let mut oracle_schema =
        PathBuf::from("../public-journal-set-oracle/fin_journal_set_oracle_v1.1.0.schema.json");
    let mut output: Option<PathBuf> = None;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--candidate" => {
                candidate = PathBuf::from(args.next().ok_or("--candidate requires a path")?)
            }
            "--candidate-schema" => {
                candidate_schema =
                    PathBuf::from(args.next().ok_or("--candidate-schema requires a path")?)
            }
            "--oracle" => oracle = PathBuf::from(args.next().ok_or("--oracle requires a path")?),
            "--oracle-schema" => {
                oracle_schema = PathBuf::from(args.next().ok_or("--oracle-schema requires a path")?)
            }
            "--out" => output = Some(PathBuf::from(args.next().ok_or("--out requires a path")?)),
            "--help" | "-h" => {
                println!(
                    "FIN-JOURNAL-SET verifier: --candidate --candidate-schema --oracle --oracle-schema --out"
                );
                return Ok(());
            }
            unknown => return Err(format!("unknown argument: {unknown}").into()),
        }
    }

    let candidate = fs::read(candidate)?;
    let candidate_schema = fs::read(candidate_schema)?;
    let oracle = fs::read(oracle)?;
    let oracle_schema = fs::read(oracle_schema)?;
    let report = verify(&candidate, &candidate_schema, &oracle, &oracle_schema);
    let rendered = serde_json::to_string_pretty(&report)?;
    match output {
        Some(path) => fs::write(path, format!("{rendered}\n"))?,
        None => println!("{rendered}"),
    }
    if report["status"] != "PASS" {
        std::process::exit(1);
    }
    Ok(())
}
