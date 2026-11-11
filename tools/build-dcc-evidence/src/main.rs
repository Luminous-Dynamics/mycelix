use building_dcc_evidence::validate_tracked;
use std::process::ExitCode;
fn main()->ExitCode{match validate_tracked(){Ok(n)=>{println!("suite=building_dcc_evidence_v0_1");println!("cases={n}");println!("result=all_known_answers_matched");println!("claim_ceiling=normalized_synthetic_evidence_axes_only");ExitCode::SUCCESS},Err(e)=>{eprintln!("result=invalid");eprintln!("{e}");ExitCode::FAILURE}}}
