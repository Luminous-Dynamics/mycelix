use fin_assurance_procedure_verifier::verify;
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

fn main() {
    if let Err(error) = run() {
        eprintln!("fin-assurance-procedure-verifier: {error}");
        std::process::exit(2);
    }
}
fn run() -> Result<(), Box<dyn Error>> {
    let mut input = PathBuf::from("../fin_assurance_procedure_candidates_v1.0.0.json");
    let mut schema = PathBuf::from("../FIN_ASSURANCE_PROCEDURE_INPUT_V1.schema.json");
    let mut oracle = PathBuf::from("../public-assurance-oracle/fin_assurance_procedure_oracle_v1.0.0.json");
    let mut oracle_schema = PathBuf::from("../public-assurance-oracle/fin_assurance_procedure_oracle_v1.schema.json");
    let mut output: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--input" => input = PathBuf::from(args.next().ok_or("--input requires path")?),
            "--schema" => schema = PathBuf::from(args.next().ok_or("--schema requires path")?),
            "--oracle" => oracle = PathBuf::from(args.next().ok_or("--oracle requires path")?),
            "--oracle-schema" => oracle_schema = PathBuf::from(args.next().ok_or("--oracle-schema requires path")?),
            "--out" => output = Some(PathBuf::from(args.next().ok_or("--out requires path")?)),
            "--help" | "-h" => { println!("FIN-ASSURE-PROCEDURE verifier: --input --schema --oracle --oracle-schema --out"); return Ok(()); }
            unknown => return Err(format!("unknown argument: {unknown}").into()),
        }
    }
    let input = fs::read(input)?;
    let schema = fs::read(schema)?;
    let oracle = fs::read(oracle)?;
    let oracle_schema = fs::read(oracle_schema)?;
    let report = verify(&input, &schema, &oracle, &oracle_schema);
    let rendered = serde_json::to_string_pretty(&report)?;
    match output { Some(path) => fs::write(path, format!("{rendered}\n"))?, None => println!("{rendered}") }
    if report["status"] != "PASS" { std::process::exit(1); }
    Ok(())
}
