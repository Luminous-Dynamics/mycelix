// Copyright (C) 2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use claim_ceiling_reference::{UNPINNED_LOCAL_RUN, policy_profile::evaluate_policy_request_bytes};
use std::{collections::BTreeMap, env, fs, io::Read, process};

/// Bound untrusted files before JSON parsing to make the CLI's memory use finite.
const MAX_REQUEST_BYTES: usize = 1024 * 1024;
const MAX_POLICY_PROFILE_BYTES: usize = 1024 * 1024;

const USAGE: &str = concat!(
    "usage: claim-ceiling-policy-check --input REQUEST.json ",
    "--policy PROFILE.json --expected-policy-sha256 SHA256 ",
    "[--source-commit SHA]",
);

#[derive(Debug)]
struct CliArgs {
    input_path: String,
    policy_path: String,
    expected_policy_sha256: String,
    source_commit: String,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("claim-ceiling-policy-check: {error}");
        process::exit(2);
    }
}

/// Parse a deliberately small CLI grammar: each recognized option appears at most once,
/// every option takes one value, and unknown/positional arguments are rejected.
fn parse_args(args: &[String]) -> Result<CliArgs, String> {
    let mut options = BTreeMap::<&str, &str>::new();
    let mut index = 0;

    while index < args.len() {
        let name = args[index].as_str();
        if !matches!(
            name,
            "--input" | "--policy" | "--expected-policy-sha256" | "--source-commit"
        ) {
            return Err(format!("unknown option or positional argument {name:?}; {USAGE}"));
        }

        let value = args
            .get(index + 1)
            .map(String::as_str)
            .ok_or_else(|| format!("argument {name} requires a value; {USAGE}"))?;
        if value.is_empty() || value.starts_with("--") {
            return Err(format!("argument {name} requires a non-empty value; {USAGE}"));
        }
        if options.insert(name, value).is_some() {
            return Err(format!("argument {name} must not be repeated; {USAGE}"));
        }
        index += 2;
    }

    let required = |name: &str| {
        options
            .get(name)
            .copied()
            .map(str::to_owned)
            .ok_or_else(|| format!("required argument {name} is missing; {USAGE}"))
    };

    Ok(CliArgs {
        input_path: required("--input")?,
        policy_path: required("--policy")?,
        expected_policy_sha256: required("--expected-policy-sha256")?,
        source_commit: options
            .get("--source-commit")
            .copied()
            .unwrap_or(UNPINNED_LOCAL_RUN)
            .to_owned(),
    })
}

/// Read no more than limit + 1 bytes, so a growing/replaced file cannot cause an
/// unbounded read between a metadata check and the actual read.
fn read_bounded(path: &str, label: &str, limit: usize) -> Result<Vec<u8>, String> {
    let file = fs::File::open(path)
        .map_err(|error| format!("cannot open {label} {path}: {error}"))?;
    let mut bytes = Vec::new();
    file.take((limit + 1) as u64)
        .read_to_end(&mut bytes)
        .map_err(|error| format!("cannot read {label} {path}: {error}"))?;
    if bytes.len() > limit {
        return Err(format!(
            "{label} {path} exceeds the {limit}-byte input limit"
        ));
    }
    Ok(bytes)
}

fn run() -> Result<(), String> {
    let args = parse_args(&env::args().skip(1).collect::<Vec<_>>())?;
    let input = read_bounded(&args.input_path, "policy request", MAX_REQUEST_BYTES)?;
    let policy = read_bounded(
        &args.policy_path,
        "policy profile",
        MAX_POLICY_PROFILE_BYTES,
    )?;
    let receipt = evaluate_policy_request_bytes(
        &input,
        &policy,
        &args.expected_policy_sha256,
        &args.source_commit,
    )?;
    println!(
        "{}",
        serde_json::to_string_pretty(&receipt)
            .map_err(|error| format!("cannot serialize policy receipt: {error}"))?
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn argv(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| (*value).to_owned()).collect()
    }

    #[test]
    fn parser_accepts_required_arguments_and_defaults_to_unpinned_run() {
        let parsed = parse_args(&argv(&[
            "--input",
            "request.json",
            "--policy",
            "profile.json",
            "--expected-policy-sha256",
            "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
        ]))
        .expect("valid invocation should parse");

        assert_eq!(parsed.input_path, "request.json");
        assert_eq!(parsed.policy_path, "profile.json");
        assert_eq!(
            parsed.expected_policy_sha256,
            "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
        );
        assert_eq!(parsed.source_commit, UNPINNED_LOCAL_RUN);
    }

    #[test]
    fn parser_accepts_an_explicit_source_commit() {
        let parsed = parse_args(&argv(&[
            "--input",
            "request.json",
            "--policy",
            "profile.json",
            "--expected-policy-sha256",
            "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
            "--source-commit",
            "ac48ea53b2f434292017bca5419cf48dcac70906",
        ]))
        .expect("valid pinned invocation should parse");

        assert_eq!(
            parsed.source_commit,
            "ac48ea53b2f434292017bca5419cf48dcac70906"
        );
    }

    #[test]
    fn parser_rejects_unknown_options_and_positional_arguments() {
        let error = parse_args(&argv(&["--mystery", "value"])).unwrap_err();
        assert!(error.contains("unknown option"));

        let error = parse_args(&argv(&["unexpected"])).unwrap_err();
        assert!(error.contains("unknown option"));
    }

    #[test]
    fn parser_rejects_duplicate_options_instead_of_silently_using_the_first() {
        let error = parse_args(&argv(&[
            "--input",
            "request.json",
            "--input",
            "attacker-request.json",
            "--policy",
            "profile.json",
            "--expected-policy-sha256",
            "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
        ]))
        .unwrap_err();
        assert!(error.contains("--input must not be repeated"));
    }

    #[test]
    fn parser_rejects_missing_values_and_option_names_used_as_values() {
        let error = parse_args(&argv(&["--input"])).unwrap_err();
        assert!(error.contains("requires a value"));

        let error = parse_args(&argv(&[
            "--input",
            "--policy",
            "profile.json",
            "--expected-policy-sha256",
            "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
        ]))
        .unwrap_err();
        assert!(error.contains("requires a non-empty value"));
    }

    #[test]
    fn bounded_reader_accepts_exact_limit_and_rejects_oversized_input() {
        let path = std::env::temp_dir().join(format!(
            "mycelix-claim-policy-bounded-read-{}",
            process::id()
        ));
        let path_string = path.to_string_lossy().into_owned();

        fs::write(&path, b"1234").expect("write test input");
        let error = read_bounded(&path_string, "test input", 3).unwrap_err();
        assert!(error.contains("exceeds the 3-byte input limit"));

        fs::write(&path, b"123").expect("replace test input");
        assert_eq!(
            read_bounded(&path_string, "test input", 3).expect("exact-size input should pass"),
            b"123"
        );
        fs::remove_file(path).expect("remove test input");
    }

    #[test]
    fn parser_requires_policy_pin_and_input() {
        let error = parse_args(&argv(&[
            "--input",
            "request.json",
            "--policy",
            "profile.json",
        ]))
        .unwrap_err();
        assert!(error.contains("--expected-policy-sha256"));

        let error = parse_args(&argv(&[
            "--policy",
            "profile.json",
            "--expected-policy-sha256",
            "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
        ]))
        .unwrap_err();
        assert!(error.contains("--input"));
    }
}
