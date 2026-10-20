// Copyright (C) 2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use claim_ceiling_reference::{UNPINNED_LOCAL_RUN, policy_profile::evaluate_policy_request_bytes};
use std::{env, fs, process};

fn main() {
    if let Err(error) = run() {
        eprintln!("claim-ceiling-policy-check: {error}");
        process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    let input_index = args
        .iter()
        .position(|arg| arg == "--input")
        .ok_or_else(|| {
            "usage: claim-ceiling-policy-check --input REQUEST.json [--source-commit SHA]"
                .to_owned()
        })?;
    let input_path = args
        .get(input_index + 1)
        .filter(|value| !value.starts_with("--"))
        .ok_or_else(|| "--input requires a path".to_owned())?;
    let source_commit = match args.iter().position(|arg| arg == "--source-commit") {
        Some(index) => args
            .get(index + 1)
            .filter(|value| !value.starts_with("--"))
            .cloned()
            .ok_or_else(|| "--source-commit requires a value".to_owned())?,
        None => UNPINNED_LOCAL_RUN.to_owned(),
    };
    let input = fs::read(input_path)
        .map_err(|error| format!("cannot read policy request {input_path}: {error}"))?;
    let receipt = evaluate_policy_request_bytes(&input, &source_commit)?;
    println!(
        "{}",
        serde_json::to_string_pretty(&receipt)
            .map_err(|error| format!("cannot serialize policy receipt: {error}"))?
    );
    Ok(())
}
