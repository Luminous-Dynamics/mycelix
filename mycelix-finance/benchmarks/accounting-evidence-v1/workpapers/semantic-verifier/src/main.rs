use fin_workpaper_dag_verifier::verify;
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

fn main() {
    if let Err(error) = run() {
        eprintln!("fin-workpaper-dag-verifier: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut input = PathBuf::from("../examples/acc-syn-0001-minimal-dag-v0.1.json");
    let mut schema = PathBuf::from("../WORKPAPER_DAG_V0.1.schema.json");
    let mut output: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--input" => input = PathBuf::from(args.next().ok_or("--input requires path")?),
            "--schema" => schema = PathBuf::from(args.next().ok_or("--schema requires path")?),
            "--out" => output = Some(PathBuf::from(args.next().ok_or("--out requires path")?)),
            "--help" | "-h" => {
                println!("FIN-ACC workpaper DAG verifier: --input PATH --schema PATH --out PATH");
                return Ok(());
            }
            unknown => return Err(format!("unknown argument: {unknown}").into()),
        }
    }
    let input_bytes = fs::read(input)?;
    let schema_bytes = fs::read(schema)?;
    let report = verify(&input_bytes, &schema_bytes);
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
