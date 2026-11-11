use building_measurement_stream::validate_tracked;
use std::process::ExitCode;
fn main()->ExitCode{
 match validate_tracked(){
  Ok(n)=>{println!("suite=building_measurement_stream_v0_1");println!("cases={n}");println!("result=all_known_answers_matched");println!("claim_ceiling=synthetic_stream_ordering_and_replay_conformance_only");ExitCode::SUCCESS}
  Err(e)=>{eprintln!("result=invalid");eprintln!("{e}");ExitCode::FAILURE}
 }
}
