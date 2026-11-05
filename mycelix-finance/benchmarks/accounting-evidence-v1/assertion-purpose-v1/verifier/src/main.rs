use fin_acc_assertion_purpose_verifier::verify;
use std::{env,error::Error,fs,path::PathBuf};
fn main(){if let Err(e)=run(){eprintln!("fin-acc-assertion-purpose-verifier: {e}");std::process::exit(2);}}
fn run()->Result<(),Box<dyn Error>>{
 let mut input=PathBuf::from("../assertion_purpose_cases_v1.0.0.json");
 let mut schema=PathBuf::from("../assertion_purpose_input_v1.schema.json");
 let mut oracle=PathBuf::from("../public-reference-oracle/assertion_purpose_oracle_v1.0.0.json");
 let mut oracle_schema=PathBuf::from("../public-reference-oracle/assertion_purpose_oracle_v1.schema.json");
 let mut output:Option<PathBuf>=None;
 let mut args=env::args().skip(1);
 while let Some(arg)=args.next(){match arg.as_str(){
  "--input"=>input=PathBuf::from(args.next().ok_or("--input requires a path")?),
  "--schema"=>schema=PathBuf::from(args.next().ok_or("--schema requires a path")?),
  "--oracle"=>oracle=PathBuf::from(args.next().ok_or("--oracle requires a path")?),
  "--oracle-schema"=>oracle_schema=PathBuf::from(args.next().ok_or("--oracle-schema requires a path")?),
  "--out"=>output=Some(PathBuf::from(args.next().ok_or("--out requires a path")?)),
  "--help"|"-h"=>{println!("FIN-ACC assertion-purpose verifier\n--input PATH --schema PATH --oracle PATH --oracle-schema PATH --out PATH");return Ok(())},
  other=>return Err(format!("unknown argument: {other}").into())
 }}
 let report=verify(&fs::read(input)?,&fs::read(schema)?,&fs::read(oracle)?,&fs::read(oracle_schema)?);
 let rendered=serde_json::to_string_pretty(&report)?;
 if let Some(path)=output{fs::write(path,format!("{rendered}\n"))?}else{println!("{rendered}")}
 if report["status"]!="PASS"{std::process::exit(1)}
 Ok(())
}
