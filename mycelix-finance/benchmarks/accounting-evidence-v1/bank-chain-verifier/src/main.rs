use fin_bank_statement_chain_verifier::verify;
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

fn main() {
    if let Err(error) = run() {
        eprintln!("fin-bank-statement-chain-verifier: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut input = PathBuf::from("../bank_statement_chain_candidates_v1.0.1.json");
    let mut schema = PathBuf::from("../bank_statement_chain_input_v1.0.1.schema.json");
    let mut oracle =
        PathBuf::from("../public-reference-oracle/bank_statement_chain_oracle_v1.0.1.json");
    let mut oracle_schema =
        PathBuf::from("../public-reference-oracle/bank_statement_chain_oracle_v1.0.1.schema.json");
    let mut output: Option<PathBuf> = None;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--input" => input = PathBuf::from(args.next().ok_or("--input requires a path")?),
            "--schema" => schema = PathBuf::from(args.next().ok_or("--schema requires a path")?),
            "--oracle" => oracle = PathBuf::from(args.next().ok_or("--oracle requires a path")?),
            "--oracle-schema" => {
                oracle_schema = PathBuf::from(args.next().ok_or("--oracle-schema requires a path")?)
            }
            "--out" => output = Some(PathBuf::from(args.next().ok_or("--out requires a path")?)),
            "--help" | "-h" => {
                println!(
                    "FIN-BANK-CHAIN verifier\n\n--input PATH --schema PATH --oracle PATH --oracle-schema PATH --out PATH"
                );
                return Ok(());
            }
            unknown => return Err(format!("unknown argument: {unknown}").into()),
        }
    }

    let input_bytes = fs::read(input)?;
    let schema_bytes = fs::read(schema)?;
    let oracle_bytes = fs::read(oracle)?;
    let oracle_schema_bytes = fs::read(oracle_schema)?;
    let report = verify(
        &input_bytes,
        &schema_bytes,
        &oracle_bytes,
        &oracle_schema_bytes,
    );
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
