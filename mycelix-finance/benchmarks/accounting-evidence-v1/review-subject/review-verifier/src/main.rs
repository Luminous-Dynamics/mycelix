use fin_accounting_review_verifier::verify;
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

fn main() {
    if let Err(error) = run() {
        eprintln!("fin-accounting-review-verifier: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut input = PathBuf::from("../fixtures/qualified-structural-review.example.json");
    let mut schema = PathBuf::from("../FIN_ACCOUNTING_REVIEW_V1.0.0.schema.json");
    let mut output: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--input" => input = PathBuf::from(args.next().ok_or("--input requires path")?),
            "--schema" => schema = PathBuf::from(args.next().ok_or("--schema requires path")?),
            "--out" => output = Some(PathBuf::from(args.next().ok_or("--out requires path")?)),
            "--help" | "-h" => {
                println!("FIN-ACC-REVIEW verifier: --input PATH --schema PATH --out PATH");
                return Ok(());
            }
            unknown => return Err(format!("unknown argument: {unknown}").into()),
        }
    }
    let input = fs::read(input)?;
    let schema = fs::read(schema)?;
    let report = verify(&input, &schema);
    let rendered = serde_json::to_string_pretty(&report)?;
    match output {
        Some(path) => fs::write(path, format!("{rendered}\n"))?,
        None => println!("{rendered}"),
    }
    Ok(())
}
