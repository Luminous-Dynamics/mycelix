// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_m0_receipt_bound_launch::{
    execute_receipt_bound_m0_capsule, verify_materialized_m0_capsule,
    MaterializedCapsuleExpectation, ReceiptBoundLaunchError,
};
use serde::{Deserialize, Serialize};
use std::{
    env,
    ffi::OsString,
    fs::{self, OpenOptions},
    io::{Read, Write},
    os::unix::fs::{OpenOptionsExt, PermissionsExt},
    path::{Path, PathBuf},
    process::ExitCode,
};
use thiserror::Error;

const EXPECTATION_SCHEMA_VERSION: u16 = 1;
const RESULT_SCHEMA_VERSION: u16 = 1;
const MAX_EXPECTATION_BYTES: usize = 64 * 1024;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ExpectationFileV1 {
    schema_version: u16,
    construction_commitment: Digest,
    execution_subject: Digest,
    execution_spec: Digest,
    run_plan: Digest,
}

impl ExpectationFileV1 {
    fn validate(self) -> Result<MaterializedCapsuleExpectation, CliError> {
        if self.schema_version != EXPECTATION_SCHEMA_VERSION {
            return Err(CliError::UnsupportedExpectationVersion(self.schema_version));
        }
        for (field, digest) in [
            ("construction_commitment", &self.construction_commitment),
            ("execution_subject", &self.execution_subject),
            ("execution_spec", &self.execution_spec),
            ("run_plan", &self.run_plan),
        ] {
            if digest.algorithm() != DigestAlgorithm::Sha256 {
                return Err(CliError::UnexpectedDigestAlgorithm(field));
            }
        }
        Ok(MaterializedCapsuleExpectation::new(
            self.construction_commitment,
            self.execution_subject,
            self.execution_spec,
            self.run_plan,
        ))
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum Command {
    VerifyMaterialized {
        root: PathBuf,
        expectation: PathBuf,
    },
    RunMaterialized {
        root: PathBuf,
        expectation: PathBuf,
    },
    Help,
}

#[derive(Serialize)]
struct VerifyResult {
    schema_version: u16,
    operation: &'static str,
    status: &'static str,
    root: String,
    expectation_file_digest: Digest,
    construction_commitment: Digest,
    execution_subject: Digest,
    execution_spec: Digest,
    run_plan: Digest,
    materialization_evidence: Digest,
    semantic_receipt: Digest,
    receipt_file: Digest,
    launch_manifest: Digest,
}

#[derive(Serialize)]
struct RunResult {
    schema_version: u16,
    operation: &'static str,
    status: &'static str,
    root: String,
    expectation_file_digest: Digest,
    construction_commitment: Digest,
    execution_subject: Digest,
    execution_spec: Digest,
    run_plan: Digest,
    materialization_evidence: Digest,
    host_evidence: Digest,
    receipt_bound_run_evidence: Digest,
    child_pid: u32,
    pidfd_observed_exit: bool,
}

fn main() -> ExitCode {
    match run(env::args_os()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("mycelix-forge: {error}");
            if matches!(error, CliError::InvalidArguments(_)) {
                eprintln!();
                eprintln!("{}", usage());
            }
            ExitCode::from(2)
        }
    }
}

fn run(args: impl IntoIterator<Item = OsString>) -> Result<(), CliError> {
    match parse_args(args)? {
        Command::Help => {
            println!("{}", usage());
            Ok(())
        }
        Command::VerifyMaterialized { root, expectation } => {
            let (expected, expectation_file_digest) = load_expectation(&expectation)?;
            let verified = verify_materialized_m0_capsule(&root, &expected)?;
            let output = VerifyResult {
                schema_version: RESULT_SCHEMA_VERSION,
                operation: "m0.verify-materialized",
                status: "verified",
                root: path_string(verified.root())?,
                expectation_file_digest,
                construction_commitment: expected.construction_commitment().clone(),
                execution_subject: expected.execution_subject().clone(),
                execution_spec: expected.execution_spec().clone(),
                run_plan: expected.run_plan().clone(),
                materialization_evidence: verified.evidence_digest().clone(),
                semantic_receipt: verified.receipt_digest().clone(),
                receipt_file: verified.receipt_file_digest().clone(),
                launch_manifest: verified.launch_manifest_digest().clone(),
            };
            write_json(&output)
        }
        Command::RunMaterialized { root, expectation } => {
            let (expected, expectation_file_digest) = load_expectation(&expectation)?;
            let completed = execute_receipt_bound_m0_capsule(&root, &expected)?;
            let output = RunResult {
                schema_version: RESULT_SCHEMA_VERSION,
                operation: "m0.run-materialized",
                status: "executed",
                root: path_string(completed.materialized().root())?,
                expectation_file_digest,
                construction_commitment: expected.construction_commitment().clone(),
                execution_subject: expected.execution_subject().clone(),
                execution_spec: expected.execution_spec().clone(),
                run_plan: expected.run_plan().clone(),
                materialization_evidence: completed.materialized().evidence_digest().clone(),
                host_evidence: completed.host().evidence_digest().clone(),
                receipt_bound_run_evidence: completed.evidence_digest().clone(),
                child_pid: completed.host().child_pid(),
                pidfd_observed_exit: completed.host().pidfd_observed_exit(),
            };
            write_json(&output)
        }
    }
}

fn parse_args(args: impl IntoIterator<Item = OsString>) -> Result<Command, CliError> {
    let mut args = args.into_iter();
    let _program = args.next();
    let Some(first) = args.next() else {
        return Err(CliError::InvalidArguments("missing command".into()));
    };
    let first = token(&first)?;
    if first == "--help" || first == "-h" || first == "help" {
        if args.next().is_some() {
            return Err(CliError::InvalidArguments(
                "help does not accept additional arguments".into(),
            ));
        }
        return Ok(Command::Help);
    }
    if first != "m0" {
        return Err(CliError::InvalidArguments(format!(
            "unknown top-level command: {first}"
        )));
    }
    let sub = args
        .next()
        .ok_or_else(|| CliError::InvalidArguments("missing m0 subcommand".into()))?;
    let sub = token(&sub)?;
    let (root, expectation) = parse_required_paths(args)?;
    match sub.as_str() {
        "verify-materialized" => Ok(Command::VerifyMaterialized { root, expectation }),
        "run-materialized" => Ok(Command::RunMaterialized { root, expectation }),
        other => Err(CliError::InvalidArguments(format!(
            "unknown m0 subcommand: {other}"
        ))),
    }
}

fn parse_required_paths(
    args: impl IntoIterator<Item = OsString>,
) -> Result<(PathBuf, PathBuf), CliError> {
    let mut args = args.into_iter();
    let mut root = None;
    let mut expectation = None;
    while let Some(flag) = args.next() {
        let flag = token(&flag)?;
        let value = args.next().ok_or_else(|| {
            CliError::InvalidArguments(format!("missing value for {flag}"))
        })?;
        match flag.as_str() {
            "--root" => {
                if root.replace(PathBuf::from(value)).is_some() {
                    return Err(CliError::InvalidArguments("duplicate --root".into()));
                }
            }
            "--expectation" => {
                if expectation.replace(PathBuf::from(value)).is_some() {
                    return Err(CliError::InvalidArguments(
                        "duplicate --expectation".into(),
                    ));
                }
            }
            other => {
                return Err(CliError::InvalidArguments(format!(
                    "unknown option: {other}"
                )))
            }
        }
    }
    Ok((
        root.ok_or_else(|| CliError::InvalidArguments("missing --root".into()))?,
        expectation
            .ok_or_else(|| CliError::InvalidArguments("missing --expectation".into()))?,
    ))
}

fn load_expectation(
    path: &Path,
) -> Result<(MaterializedCapsuleExpectation, Digest), CliError> {
    let metadata = fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(CliError::InvalidExpectationFile(path.to_path_buf()));
    }
    let mode = metadata.permissions().mode() & 0o777;
    if mode & 0o022 != 0 {
        return Err(CliError::ExpectationFileWritableByOthers {
            path: path.to_path_buf(),
            mode,
        });
    }
    let mut file = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open(path)?;
    let limit = u64::try_from(MAX_EXPECTATION_BYTES)
        .map_err(|_| CliError::ExpectationTooLarge(path.to_path_buf()))?
        .saturating_add(1);
    let mut bytes = Vec::new();
    file.by_ref().take(limit).read_to_end(&mut bytes)?;
    if bytes.len() > MAX_EXPECTATION_BYTES {
        return Err(CliError::ExpectationTooLarge(path.to_path_buf()));
    }
    let transport_digest = Digest::of_bytes(DigestAlgorithm::Sha256, &bytes);
    let expectation: ExpectationFileV1 = serde_json::from_slice(&bytes)?;
    Ok((expectation.validate()?, transport_digest))
}

fn write_json(value: &impl Serialize) -> Result<(), CliError> {
    let stdout = std::io::stdout();
    let mut lock = stdout.lock();
    serde_json::to_writer_pretty(&mut lock, value)?;
    writeln!(&mut lock)?;
    lock.flush()?;
    Ok(())
}

fn path_string(path: &Path) -> Result<String, CliError> {
    path.to_str()
        .map(ToOwned::to_owned)
        .ok_or_else(|| CliError::NonUtf8Path(path.to_path_buf()))
}

fn token(value: &OsString) -> Result<String, CliError> {
    value
        .to_str()
        .map(ToOwned::to_owned)
        .ok_or_else(|| CliError::InvalidArguments("command token is not UTF-8".into()))
}

fn usage() -> &'static str {
    "Usage:\n  mycelix-forge m0 verify-materialized --root <capsule-dir> --expectation <expectation.json>\n  mycelix-forge m0 run-materialized    --root <capsule-dir> --expectation <expectation.json>\n\nExpectation v1 JSON fields:\n  schema_version, construction_commitment, execution_subject, execution_spec, run_plan\n\nThe expectation file is an external trust input. It is never inferred from the materialization receipt."
}

#[derive(Debug, Error)]
enum CliError {
    #[error(transparent)]
    Launch(#[from] ReceiptBoundLaunchError),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error("invalid arguments: {0}")]
    InvalidArguments(String),
    #[error("invalid expectation file: {0:?}")]
    InvalidExpectationFile(PathBuf),
    #[error("expectation file is writable by group/other: {path:?} mode={mode:o}")]
    ExpectationFileWritableByOthers { path: PathBuf, mode: u32 },
    #[error("expectation file exceeds 64 KiB: {0:?}")]
    ExpectationTooLarge(PathBuf),
    #[error("unsupported expectation schema version: {0}")]
    UnsupportedExpectationVersion(u16),
    #[error("expectation field {0} does not use the exact M0 SHA-256 profile")]
    UnexpectedDigestAlgorithm(&'static str),
    #[error("path is not UTF-8: {0:?}")]
    NonUtf8Path(PathBuf),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(parts: &[&str]) -> Result<Command, CliError> {
        parse_args(parts.iter().map(|part| OsString::from(*part)))
    }

    #[test]
    fn verify_command_requires_explicit_root_and_expectation() {
        let command = parse(&[
            "mycelix-forge",
            "m0",
            "verify-materialized",
            "--root",
            "/tmp/capsule",
            "--expectation",
            "/tmp/expectation.json",
        ])
        .unwrap();
        assert_eq!(
            command,
            Command::VerifyMaterialized {
                root: PathBuf::from("/tmp/capsule"),
                expectation: PathBuf::from("/tmp/expectation.json"),
            }
        );
    }

    #[test]
    fn run_command_accepts_option_order_independently() {
        let command = parse(&[
            "mycelix-forge",
            "m0",
            "run-materialized",
            "--expectation",
            "/tmp/expectation.json",
            "--root",
            "/tmp/capsule",
        ])
        .unwrap();
        assert_eq!(
            command,
            Command::RunMaterialized {
                root: PathBuf::from("/tmp/capsule"),
                expectation: PathBuf::from("/tmp/expectation.json"),
            }
        );
    }

    #[test]
    fn duplicate_or_unknown_options_fail_closed() {
        assert!(parse(&[
            "mycelix-forge",
            "m0",
            "verify-materialized",
            "--root",
            "a",
            "--root",
            "b",
            "--expectation",
            "c",
        ])
        .is_err());
        assert!(parse(&[
            "mycelix-forge",
            "m0",
            "verify-materialized",
            "--root",
            "a",
            "--expectation",
            "b",
            "--unsafe",
            "yes",
        ])
        .is_err());
    }

    #[test]
    fn help_is_explicit_command() {
        assert_eq!(
            parse(&["mycelix-forge", "--help"]).unwrap(),
            Command::Help
        );
    }
}
