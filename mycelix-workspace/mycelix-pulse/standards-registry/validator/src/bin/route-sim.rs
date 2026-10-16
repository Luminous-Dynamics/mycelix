use mycelix_pulse_standards_registry_check::{
    evaluation::{evaluate_correlation_sweep, evaluate_paired_seeds},
    simulation::simulate_corpus,
};
use serde_json::Value;
use std::{
    env, fs,
    path::PathBuf,
    process::{Command, ExitCode},
};

fn main() -> ExitCode {
    let mut trace_path = PathBuf::from("benchmarks/traces.v1.json");
    let mut messages = None;
    let mut seed = None;
    let mut seed_count = 30_u64;
    let mut candidate = String::from("sequential_failover");
    let mut evaluate = false;
    let mut correlation_sweep = false;
    let mut subject_sha = None::<String>;
    let mut input_manifest_sha256 = None::<String>;
    let mut input_manifest_path = None::<PathBuf>;
    let mut seed_count_explicit = false;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--traces" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --traces requires a path");
                    return ExitCode::from(2);
                };
                trace_path = PathBuf::from(value);
            }
            "--messages" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --messages requires a number");
                    return ExitCode::from(2);
                };
                match value.parse::<u64>() {
                    Ok(value) if value > 0 => messages = Some(value),
                    _ => {
                        eprintln!("error: --messages must be a positive integer");
                        return ExitCode::from(2);
                    }
                }
            }
            "--seed" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --seed requires a number");
                    return ExitCode::from(2);
                };
                match value.parse::<u64>() {
                    Ok(value) => seed = Some(value),
                    Err(_) => {
                        eprintln!("error: --seed must be a non-negative integer");
                        return ExitCode::from(2);
                    }
                }
            }
            "--evaluate" => evaluate = true,
            "--correlation-sweep" => correlation_sweep = true,
            "--seed-count" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --seed-count requires a number");
                    return ExitCode::from(2);
                };
                match value.parse::<u64>() {
                    Ok(value) => {
                        seed_count = value;
                        seed_count_explicit = true;
                    }
                    Err(_) => {
                        eprintln!("error: --seed-count must be a non-negative integer");
                        return ExitCode::from(2);
                    }
                }
            }
            "--candidate" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --candidate requires a strategy");
                    return ExitCode::from(2);
                };
                candidate = value;
            }
            "--subject-sha" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --subject-sha requires a commit SHA");
                    return ExitCode::from(2);
                };
                subject_sha = Some(value);
            }
            "--input-manifest-sha256" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --input-manifest-sha256 requires a SHA-256 digest");
                    return ExitCode::from(2);
                };
                input_manifest_sha256 = Some(value);
            }
            "--input-manifest" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --input-manifest requires a manifest path");
                    return ExitCode::from(2);
                };
                input_manifest_path = Some(PathBuf::from(value));
            }
            "--help" | "-h" => {
                println!("Usage: route-sim [--traces PATH] [--messages COUNT] [--seed N]");
                println!("       route-sim --evaluate [--correlation-sweep] [--traces PATH] [--messages COUNT]");
                println!(concat!(
                    "                 [--seed FIRST] ",
                    "[--seed-count COUNT] [--candidate STRATEGY]"
                ));
                println!("                 [--correlation-sweep]");
                println!(concat!(
                    "                 --subject-sha SHA --input-manifest PATH ",
                    "--input-manifest-sha256 SHA256"
                ));
                println!();
                println!(concat!(
                    "Strategies: preferred_single_path, ",
                    "sequential_failover, parallel_redundancy"
                ));
                println!(concat!(
                    "Evaluation compares paired synthetic seeds ",
                    "on the declared holdout split."
                ));
                println!(concat!(
                    "--correlation-sweep varies the declared shared-draw ",
                    "mixture on resilience scenarios; it is not an empirical correlation fit."
                ));
                println!(concat!(
                    "Neither mode establishes authenticated delivery ",
                    "or hardware qualification."
                ));
                return ExitCode::SUCCESS;
            }
            unknown => {
                eprintln!("error: unknown argument '{unknown}'");
                return ExitCode::from(2);
            }
        }
    }

    if !evaluate
        && (seed_count_explicit
            || correlation_sweep
            || subject_sha.is_some()
            || input_manifest_sha256.is_some()
            || input_manifest_path.is_some())
    {
        eprintln!(concat!(
            "error: --seed-count, --correlation-sweep, --subject-sha and ",
            "--input-manifest, --input-manifest-sha256 require --evaluate"
        ));
        return ExitCode::from(2);
    }

    let text = match fs::read_to_string(&trace_path) {
        Ok(text) => text,
        Err(error) => {
            eprintln!("error: cannot read {}: {error}", trace_path.display());
            return ExitCode::from(2);
        }
    };
    let corpus: Value = match serde_json::from_str(&text) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("error: invalid trace JSON in {}: {error}", trace_path.display());
            return ExitCode::from(2);
        }
    };

    if evaluate {
        let Some(subject_sha) = subject_sha else {
            eprintln!("error: --evaluate requires --subject-sha");
            return ExitCode::from(2);
        };
        let Some(input_manifest_sha256) = input_manifest_sha256 else {
            eprintln!("error: --evaluate requires --input-manifest-sha256");
            return ExitCode::from(2);
        };
        let Some(input_manifest_path) = input_manifest_path else {
            eprintln!("error: --evaluate requires --input-manifest PATH");
            return ExitCode::from(2);
        };
        if !valid_subject_sha(&subject_sha) {
            eprintln!("error: --subject-sha must be a 40- or 64-character hexadecimal commit SHA");
            return ExitCode::from(2);
        }
        if !is_hex_digest(&input_manifest_sha256) {
            eprintln!(concat!(
                "error: --input-manifest-sha256 must be a ",
                "64-character hexadecimal SHA-256 digest"
            ));
            return ExitCode::from(2);
        }

        let (verified_subject_sha, verified_manifest_sha256) =
            match verify_evidence_binding(
                &subject_sha,
                &input_manifest_path,
                &input_manifest_sha256,
            ) {
                Ok(binding) => binding,
                Err(error) => {
                    eprintln!("INVALID evidence binding: {error}");
                    return ExitCode::FAILURE;
                }
            };

        let first_seed = seed.or_else(|| corpus.get("seed").and_then(Value::as_u64)).unwrap_or(0);
        let evaluation = if correlation_sweep {
            evaluate_correlation_sweep(
                &corpus,
                messages,
                first_seed,
                seed_count,
                &candidate,
            )
        } else {
            evaluate_paired_seeds(
                &corpus,
                messages,
                first_seed,
                seed_count,
                &candidate,
            )
        };
        let mut report = match evaluation {
            Ok(report) => report,
            Err(errors) => {
                eprintln!("INVALID evaluation inputs:");
                for error in errors {
                    eprintln!("  - {error}");
                }
                return ExitCode::FAILURE;
            }
        };
        report["subject_sha"] = Value::String(verified_subject_sha);
        report["input_manifest_path"] =
            Value::String(input_manifest_path.display().to_string());
        report["input_manifest_sha256"] = Value::String(verified_manifest_sha256);
        report["evidence_manifest_verified_by_cli"] = Value::Bool(true);
        println!("{}", serde_json::to_string_pretty(&report).expect("report serializes"));
        return ExitCode::SUCCESS;
    }

    let report = match simulate_corpus(&corpus, messages, seed) {
        Ok(report) => report,
        Err(errors) => {
            eprintln!("INVALID trace corpus:");
            for error in errors {
                eprintln!("  - {error}");
            }
            return ExitCode::FAILURE;
        }
    };
    println!("{}", serde_json::to_string_pretty(&report).expect("report serializes"));
    ExitCode::SUCCESS
}

fn verify_evidence_binding(
    requested_subject_sha: &str,
    manifest_path: &PathBuf,
    requested_manifest_sha256: &str,
) -> Result<(String, String), String> {
    let head = Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .map_err(|error| format!("cannot execute git rev-parse HEAD: {error}"))?;
    if !head.status.success() {
        return Err("git rev-parse HEAD failed; run evidence mode inside a Git checkout".to_owned());
    }
    let actual_subject = String::from_utf8_lossy(&head.stdout).trim().to_owned();
    if !actual_subject.eq_ignore_ascii_case(requested_subject_sha) {
        return Err(format!(
            "supplied subject SHA does not match checked-out HEAD (actual {actual_subject})"
        ));
    }

    let check = Command::new("sha256sum")
        .arg("--check")
        .arg("--strict")
        .arg(manifest_path)
        .output()
        .map_err(|error| format!("cannot execute sha256sum --check: {error}"))?;
    if !check.status.success() {
        let stdout = String::from_utf8_lossy(&check.stdout);
        let stderr = String::from_utf8_lossy(&check.stderr);
        return Err(format!(
            "input manifest did not verify all listed inputs: {}{}",
            stdout, stderr
        ));
    }

    let digest_output = Command::new("sha256sum")
        .arg(manifest_path)
        .output()
        .map_err(|error| format!("cannot execute sha256sum on input manifest: {error}"))?;
    if !digest_output.status.success() {
        return Err("sha256sum could not digest the input manifest".to_owned());
    }
    let digest_line = String::from_utf8_lossy(&digest_output.stdout);
    let actual_digest = digest_line
        .split_whitespace()
        .next()
        .ok_or_else(|| "sha256sum returned an empty manifest digest".to_owned())?
        .to_owned();
    if !actual_digest.eq_ignore_ascii_case(requested_manifest_sha256) {
        return Err(format!(
            "supplied manifest digest does not match the manifest file (actual {actual_digest})"
        ));
    }
    Ok((actual_subject, actual_digest))
}

fn valid_subject_sha(value: &str) -> bool {
    (value.len() == 40 || value.len() == 64)
        && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn is_hex_digest(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sha256(path: &PathBuf) -> String {
        let output = Command::new("sha256sum")
            .arg(path)
            .output()
            .expect("sha256sum must be available for evidence verification tests");
        assert!(output.status.success());
        String::from_utf8_lossy(&output.stdout)
            .split_whitespace()
            .next()
            .expect("sha256sum must emit a digest")
            .to_owned()
    }

    #[test]
    fn evidence_binding_checks_head_manifest_contents_and_digest() {
        let head = Command::new("git")
            .args(["rev-parse", "HEAD"])
            .output()
            .expect("git must be available for evidence verification tests");
        assert!(head.status.success());
        let subject = String::from_utf8_lossy(&head.stdout).trim().to_owned();

        let source = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/evaluation.rs");
        let source_manifest = Command::new("sha256sum")
            .arg(&source)
            .output()
            .expect("sha256sum must be available");
        assert!(source_manifest.status.success());

        let manifest = env::temp_dir().join(format!(
            "pulse-route-sim-evidence-{}.sha256",
            std::process::id()
        ));
        fs::write(&manifest, source_manifest.stdout).expect("write temporary hash manifest");
        let digest = sha256(&manifest);

        let verified = verify_evidence_binding(&subject, &manifest, &digest)
            .expect("current HEAD and an intact input manifest should verify");
        assert_eq!(verified.0, subject);

        let wrong_subject = "0".repeat(subject.len());
        assert!(
            verify_evidence_binding(&wrong_subject, &manifest, &digest).is_err(),
            "mismatched subject must fail closed"
        );

        let mut tampered = fs::read_to_string(&manifest).expect("read manifest");
        tampered.push_str(&format!("{}  {}\n", "0".repeat(64), source.display()));
        fs::write(&manifest, tampered).expect("write tampered manifest");
        let tampered_digest = sha256(&manifest);
        assert!(
            verify_evidence_binding(&subject, &manifest, &tampered_digest).is_err(),
            "a manifest with a mismatched input hash must fail closed even when its own digest matches"
        );
        let _ = fs::remove_file(manifest);
    }
}
