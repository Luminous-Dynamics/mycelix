// Copyright (C) 2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use claim_ceiling_reference::{UNPINNED_LOCAL_RUN, evaluate_request_bytes};
use std::{env, fs, process};

fn main() {
    if let Err(error) = run() {
        eprintln!("claim-ceiling-meet: {error}");
        process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    let input_index = args.iter().position(|arg| arg == "--input")
        .ok_or_else(|| "usage: claim-ceiling-meet --input REQUEST.json [--source-commit SHA]".to_owned())?;
    let input_path = args.get(input_index + 1)
        .ok_or_else(|| "--input requires a path".to_owned())?;
    if input_path.starts_with("--") {
        return Err("--input requires a path".to_owned());
    }
    let source_commit = match args.iter().position(|arg| arg == "--source-commit") {
        Some(index) => args.get(index + 1)
            .filter(|value| !value.starts_with("--"))
            .cloned()
            .ok_or_else(|| "--source-commit requires a value".to_owned())?,
        None => UNPINNED_LOCAL_RUN.to_owned(),
    };
    let input = fs::read(input_path)
        .map_err(|error| format!("cannot read request {input_path}: {error}"))?;
    let receipt = evaluate_request_bytes(&input, &source_commit)?;
    println!("{}", serde_json::to_string_pretty(&receipt)
        .map_err(|error| format!("cannot serialize receipt: {error}"))?);
    Ok(())
}
