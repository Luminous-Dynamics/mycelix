use building_condition_applicability::validate_tracked;
use std::process::ExitCode;

fn main()->ExitCode{
 match validate_tracked(){
  Ok(count)=>{println!("suite=building_condition_applicability_v0_1");println!("cases={count}");println!("result=all_known_answers_matched");println!("claim_ceiling=synthetic_condition_envelope_match_only");ExitCode::SUCCESS}
  Err(e)=>{eprintln!("result=invalid");eprintln!("{e}");ExitCode::FAILURE}
 }
}
