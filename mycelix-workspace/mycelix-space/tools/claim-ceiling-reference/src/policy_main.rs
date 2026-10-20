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

fn option_value(args: &[String], name: &str) -> Result<Option<String>, String> {
    let Some(index) = args.iter().position(|arg| arg == name) else {
        return Ok(None);
    };
    args.get(index + 1)
        .filter(|value| !value.starts_with("--"))
        .cloned()
        .map(Some)
        .ok_or_else(|| format!("argument {name} requires a value"))
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    let input_path = option_value(&args, "--input")?.ok_or_else(|| {
        "usage: claim-ceiling-policy-check --input REQUEST.json --policy PROFILE.json --expected-policy-sha256 SHA256 [--source-commit SHA]".to_owned()
    })?;
    let policy_path = option_value(&args, "--policy")?.ok_or_else(|| {
        "usage: claim-ceiling-policy-check --input REQUEST.json --policy PROFILE.json --expected-policy-sha256 SHA256 [--source-commit SHA]".to_owned()
    })?;
    let expected_policy_sha256 = option_value(&args, "--expected-policy-sha256")?
        .ok_or_else(|| {
            "--expected-policy-sha256 is required; pin the profile through a separate trusted channel"
                .to_owned()
        })?;
    let source_commit =
        option_value(&args, "--source-commit")?.unwrap_or_else(|| UNPINNED_LOCAL_RUN.to_owned());

    let input = fs::read(&input_path)
        .map_err(|error| format!("cannot read policy request {input_path}: {error}"))?;
    let policy = fs::read(&policy_path)
        .map_err(|error| format!("cannot read policy profile {policy_path}: {error}"))?;
    let receipt =
        evaluate_policy_request_bytes(&input, &policy, &expected_policy_sha256, &source_commit)?;
    println!(
        "{}",
        serde_json::to_string_pretty(&receipt)
            .map_err(|error| format!("cannot serialize policy receipt: {error}"))?
    );
    Ok(())
}
