use furniture_compiler::{compile_json, verify_integrity, MAX_INPUT_BYTES};
use std::io::{self, Read};

fn main() {
    if let Err(message) = run() {
        eprintln!("furniture-compile: {message}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let mut input = String::new();
    io::stdin()
        .take((MAX_INPUT_BYTES + 1) as u64)
        .read_to_string(&mut input)
        .map_err(|error| format!("could not read UTF-8 input: {error}"))?;

    let packet = compile_json(&input).map_err(|error| error.to_string())?;
    if !verify_integrity(&packet).map_err(|error| error.to_string())? {
        return Err("internal packet digest verification failed".to_owned());
    }

    let output = serde_json::to_string_pretty(&packet)
        .map_err(|error| format!("could not serialize packet: {error}"))?;
    println!("{output}");
    Ok(())
}
