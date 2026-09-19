// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D3B2C2C: concrete execution of the frozen Forge M0 guest plan.
//!
//! The runner is deliberately policy-poor: protocol semantics live in the
//! guest-plan/transcript/envelope contracts. This crate performs the required
//! filesystem/process operations using exact tool paths from `GuestToolMapV1`.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_gittuf_adapter::{
    AdapterError, GittufAdapter, GittufInvocation, SystemCommandRunner,
};
use mycelix_forge_gittuf_bundle::{
    fingerprint_file, parse_bundle_heads, require_bundle_v3, BundleRef, OfflineBundleError,
    OfflineBundleManifest,
};
use mycelix_forge_gittuf_policy_inventory::{
    GittufPolicyInventoryCollector, PolicyInventoryError, SystemByteCommandRunner,
};
use mycelix_forge_gittuf_trust_profile::{
    local_embedded_keys_v1, qualify_local_key_profile, TrustProfileError,
};
use mycelix_forge_guest_envelope::{
    qualify_guest_evidence_envelope, GuestEnvelopeError, GuestEvidenceEnvelopeV1,
};
use mycelix_forge_guest_plan::{
    GuestPhase, GuestPlanError, GuestVerificationPlanV1, GitObjectValidationPolicyV1,
    GUEST_BUNDLE_PATH, GUEST_ISOLATION_POLICY_PATH, GUEST_MANIFEST_PATH,
    GUEST_NIX_CLOSURE_PATH, GUEST_PLAN_PATH, GUEST_REPLAY_REPOSITORY,
    GUEST_RUN_CHALLENGE_PATH, GUEST_SANDBOX_INVOCATION_PATH, GUEST_TRANSCRIPT_PATH,
};
use mycelix_forge_guest_tool_map::{GuestToolMapError, GuestToolMapV1};
use mycelix_forge_guest_transcript::{
    ambient_sources_phase_subject, git_validation_phase_subject, qualify_guest_transcript,
    replay_phase_subject, transcript_output_policy, GuestPhaseEvidence, GuestTranscriptBindings,
    GuestTranscriptError, GuestTranscriptV1,
};
use mycelix_forge_linux_isolation::{
    IsolationPolicyError, LinuxIsolationPolicyV1, NixClosureManifest, VerifierInvocation,
};
use mycelix_forge_linux_isolation_collector::{
    collect_inside_evidence, IsolationCollectorError,
};
use mycelix_forge_linux_isolation_evidence::{
    expected_environment, InsideIsolationEvidence, IsolationEvidenceError,
};
use mycelix_forge_repository::{
    GitObjectAlgorithm, GitObjectId, RepositoryVerificationError,
};
use serde::{de::DeserializeOwned, Serialize};
use std::{
    ffi::OsStr,
    fs::{self, File, OpenOptions},
    io::{Read, Write},
    path::{Path, PathBuf},
    process::{Command, Output},
};
use thiserror::Error;

pub const GUEST_TOOL_MAP_PATH: &str = "/inputs/guest-tool-map.json";
pub const GUEST_ENVELOPE_PATH: &str = "/work/guest-evidence-envelope.json";
const RUN_CHALLENGE_SIZE: usize = 32;
const MAX_COMMAND_DIAGNOSTIC: usize = 8192;

pub fn run_guest_from_fixed_inputs() -> Result<GuestEvidenceEnvelopeV1, GuestRunnerError> {
    let plan: GuestVerificationPlanV1 = load_json(Path::new(GUEST_PLAN_PATH))?;
    let manifest: OfflineBundleManifest = load_json(Path::new(GUEST_MANIFEST_PATH))?;
    let isolation_policy: LinuxIsolationPolicyV1 =
        load_json(Path::new(GUEST_ISOLATION_POLICY_PATH))?;
    let closure: NixClosureManifest = load_json(Path::new(GUEST_NIX_CLOSURE_PATH))?;
    let invocation: VerifierInvocation = load_json(Path::new(GUEST_SANDBOX_INVOCATION_PATH))?;
    let tool_map: GuestToolMapV1 = load_json(Path::new(GUEST_TOOL_MAP_PATH))?;

    validate_guest_inputs(
        &plan,
        &manifest,
        &isolation_policy,
        &closure,
        &invocation,
        &tool_map,
    )?;

    let challenge = fs::read(GUEST_RUN_CHALLENGE_PATH)?;
    if challenge.len() != RUN_CHALLENGE_SIZE {
        return Err(GuestRunnerError::InvalidRunChallengeSize(challenge.len()));
    }
    let challenge_digest = Digest::of_bytes(plan.run_challenge().algorithm(), &challenge);
    if &challenge_digest != plan.run_challenge() {
        return Err(GuestRunnerError::RunChallengeMismatch);
    }

    let probe = tool_path(&tool_map, "forge-isolation-probe")?;
    let git = tool_path(&tool_map, "git")?;
    let gittuf = tool_path(&tool_map, "gittuf")?;

    let inside = run_isolation_probe(probe)?;
    let inside_digest = inside.digest(DigestAlgorithm::Sha256)?;

    replay_bundle_without_verification(git, &manifest)?;
    reject_ambient_git_sources(git)?;
    run_strict_git_fsck(git)?;

    let inventory_collector = GittufPolicyInventoryCollector::new(
        SystemByteCommandRunner,
        PathBuf::from(git),
    );
    let inventory_receipt = inventory_collector.collect(
        Path::new(GUEST_REPLAY_REPOSITORY),
        manifest.policy_state(),
    )?;
    let inventory = inventory_receipt.inventory().clone();
    let inventory_digest = inventory.digest(DigestAlgorithm::Sha256)?;
    let trust = qualify_local_key_profile(&inventory)?;
    if trust.profile_digest() != local_embedded_keys_v1().profile_digest()
        || trust.profile_digest() != plan.trust_profile()
    {
        return Err(GuestRunnerError::TrustProfileMismatch);
    }

    let gittuf_invocation = GittufInvocation::from_request(
        plan.request(),
        manifest.policy_state(),
        manifest.policy_state_digest().algorithm(),
    )?;
    let gittuf_adapter = GittufAdapter::new(
        SystemCommandRunner,
        PathBuf::from(gittuf),
        PathBuf::from(git),
    );
    let receipt = gittuf_adapter.observe(
        Path::new(GUEST_REPLAY_REPOSITORY),
        &gittuf_invocation,
    )?;
    receipt.clone().into_observation_for(plan.request())?;
    let receipt_digest = receipt.commitment(DigestAlgorithm::Sha256)?;
    if &receipt_digest != plan.expected_replay_receipt() {
        return Err(GuestRunnerError::ReplayReceiptMismatch);
    }

    let transcript = GuestTranscriptV1::new(
        plan.digest(DigestAlgorithm::Sha256)?,
        plan.execution_subject().clone(),
        plan.request_digest()?,
        plan.run_challenge().clone(),
        vec![
            GuestPhaseEvidence::new(GuestPhase::IsolationProbe, inside_digest.clone()),
            GuestPhaseEvidence::new(GuestPhase::ReplayBundle, replay_phase_subject(&plan)?),
            GuestPhaseEvidence::new(
                GuestPhase::RejectAmbientObjectSources,
                ambient_sources_phase_subject(&plan)?,
            ),
            GuestPhaseEvidence::new(
                GuestPhase::GitObjectValidation,
                git_validation_phase_subject(&plan)?,
            ),
            GuestPhaseEvidence::new(GuestPhase::PolicyTrustInventory, inventory_digest.clone()),
            GuestPhaseEvidence::new(
                GuestPhase::LocalTrustQualification,
                trust.evidence_digest().clone(),
            ),
            GuestPhaseEvidence::new(GuestPhase::GittufVerification, receipt_digest.clone()),
            GuestPhaseEvidence::new(GuestPhase::EmitTranscript, transcript_output_policy()?),
        ],
    )?;
    let bindings = GuestTranscriptBindings::new(
        inside_digest,
        inventory_digest,
        trust.evidence_digest().clone(),
        receipt_digest,
    );
    qualify_guest_transcript(&plan, &transcript, &bindings)?;

    write_json_atomic(Path::new(GUEST_TRANSCRIPT_PATH), &transcript)?;

    let envelope = GuestEvidenceEnvelopeV1::new(
        plan.digest(DigestAlgorithm::Sha256)?,
        tool_map.digest(DigestAlgorithm::Sha256)?,
        inside,
        inventory,
        receipt,
        transcript,
    );
    // Guest-side self-check catches local construction mistakes. The host must
    // still re-run this qualification after the process boundary.
    qualify_guest_evidence_envelope(&envelope, &plan, &tool_map)?;
    write_json_atomic(Path::new(GUEST_ENVELOPE_PATH), &envelope)?;
    Ok(envelope)
}

pub fn run_isolation_probe_binary() -> Result<(), GuestRunnerError> {
    let args = std::env::args().collect::<Vec<_>>();
    if args.len() != 3 || args[1] != "--policy" || args[2] != GUEST_ISOLATION_POLICY_PATH {
        return Err(GuestRunnerError::InvalidProbeArguments);
    }
    let policy: LinuxIsolationPolicyV1 = load_json(Path::new(GUEST_ISOLATION_POLICY_PATH))?;
    let evidence = collect_inside_evidence(&policy)?;
    let bytes = serde_json::to_vec(&evidence)?;
    let mut stdout = std::io::stdout().lock();
    stdout.write_all(&bytes)?;
    stdout.write_all(b"\n")?;
    stdout.flush()?;
    Ok(())
}

fn validate_guest_inputs(
    plan: &GuestVerificationPlanV1,
    manifest: &OfflineBundleManifest,
    isolation_policy: &LinuxIsolationPolicyV1,
    closure: &NixClosureManifest,
    invocation: &VerifierInvocation,
    tool_map: &GuestToolMapV1,
) -> Result<(), GuestRunnerError> {
    manifest.validate_for_request(plan.request())?;
    if &manifest.commitment(DigestAlgorithm::Sha256)? != plan.bundle_manifest() {
        return Err(GuestRunnerError::ManifestCommitmentMismatch);
    }
    if &manifest.policy_state().digest(DigestAlgorithm::Sha256)? != plan.policy_state() {
        return Err(GuestRunnerError::PolicyStateMismatch);
    }
    if &isolation_policy.digest(DigestAlgorithm::Sha256)? != plan.isolation_policy() {
        return Err(GuestRunnerError::IsolationPolicyMismatch);
    }
    if &closure.digest(DigestAlgorithm::Sha256)? != plan.nix_closure() {
        return Err(GuestRunnerError::ClosureMismatch);
    }
    if &invocation.digest(DigestAlgorithm::Sha256)? != plan.sandbox_invocation() {
        return Err(GuestRunnerError::SandboxInvocationMismatch);
    }
    let plan_digest = plan.digest(DigestAlgorithm::Sha256)?;
    if tool_map.plan_digest() != &plan_digest
        || tool_map.execution_subject() != plan.execution_subject()
        || tool_map.nix_closure() != plan.nix_closure()
    {
        return Err(GuestRunnerError::GuestToolMapMismatch);
    }
    if plan.trust_profile() != local_embedded_keys_v1().profile_digest() {
        return Err(GuestRunnerError::TrustProfileMismatch);
    }
    Ok(())
}

fn tool_path<'a>(map: &'a GuestToolMapV1, role: &str) -> Result<&'a str, GuestRunnerError> {
    map.tool(role)
        .map(|tool| tool.executable())
        .ok_or_else(|| GuestRunnerError::MissingGuestTool(role.to_owned()))
}

fn run_isolation_probe(executable: &str) -> Result<InsideIsolationEvidence, GuestRunnerError> {
    let output = run_exact(
        executable,
        &["--policy", GUEST_ISOLATION_POLICY_PATH],
        Path::new("/work"),
    )?;
    require_success(executable, &["--policy", GUEST_ISOLATION_POLICY_PATH], &output)?;
    Ok(serde_json::from_slice(&output.stdout)?)
}

fn replay_bundle_without_verification(
    git: &str,
    manifest: &OfflineBundleManifest,
) -> Result<(), GuestRunnerError> {
    let bundle = Path::new(GUEST_BUNDLE_PATH);
    require_bundle_v3(bundle)?;
    let fingerprint = fingerprint_file(bundle)?;
    if &fingerprint.digest != manifest.bundle_digest()
        || fingerprint.size != manifest.bundle_size()
    {
        return Err(GuestRunnerError::BundleArtifactMismatch);
    }

    let repo = Path::new(GUEST_REPLAY_REPOSITORY);
    if repo.exists() {
        return Err(GuestRunnerError::ReplayRepositoryAlreadyExists);
    }
    fs::create_dir(repo)?;
    let format = match manifest.object_format() {
        GitObjectAlgorithm::Sha1 => "sha1",
        GitObjectAlgorithm::Sha256 => "sha256",
    };
    let init_args = [
        "init".to_owned(),
        "--bare".to_owned(),
        "--template=".to_owned(),
        format!("--object-format={format}"),
        ".".to_owned(),
    ];
    let init = run_git_owned(git, &init_args, repo)?;
    require_success_owned(git, &init_args, &init)?;

    let verify_args = [
        "bundle".to_owned(),
        "verify".to_owned(),
        GUEST_BUNDLE_PATH.to_owned(),
    ];
    let verify = run_git_owned(git, &verify_args, repo)?;
    require_success_owned(git, &verify_args, &verify)?;

    let list_args = [
        "bundle".to_owned(),
        "list-heads".to_owned(),
        GUEST_BUNDLE_PATH.to_owned(),
    ];
    let listed = run_git_owned(git, &list_args, repo)?;
    require_success_owned(git, &list_args, &listed)?;
    let listed_text = std::str::from_utf8(&listed.stdout)
        .map_err(|_| GuestRunnerError::NonUtf8CommandOutput("bundle list-heads"))?;
    let advertised = parse_bundle_heads(listed_text, manifest.object_format())?;
    if advertised != manifest.refs() {
        return Err(GuestRunnerError::BundleRefSetMismatch);
    }

    let mut fetch_args = vec![
        "fetch".to_owned(),
        "--no-tags".to_owned(),
        "--no-write-fetch-head".to_owned(),
        GUEST_BUNDLE_PATH.to_owned(),
    ];
    for entry in manifest.refs() {
        let reference = entry.reference().as_str();
        fetch_args.push(format!("+{reference}:{reference}"));
    }
    let fetch = run_git_owned(git, &fetch_args, repo)?;
    require_success_owned(git, &fetch_args, &fetch)?;

    let imported = read_exact_refs(git, repo, manifest)?;
    if imported != manifest.refs() {
        return Err(GuestRunnerError::ImportedRefSetMismatch);
    }
    Ok(())
}

fn read_exact_refs(
    git: &str,
    repo: &Path,
    manifest: &OfflineBundleManifest,
) -> Result<Vec<BundleRef>, GuestRunnerError> {
    let mut refs = Vec::with_capacity(manifest.refs().len());
    for expected in manifest.refs() {
        let args = [
            "rev-parse".to_owned(),
            "--verify".to_owned(),
            expected.reference().to_string(),
        ];
        let output = run_git_owned(git, &args, repo)?;
        require_success_owned(git, &args, &output)?;
        let text = std::str::from_utf8(&output.stdout)
            .map_err(|_| GuestRunnerError::NonUtf8CommandOutput("rev-parse"))?;
        refs.push(BundleRef::new(
            expected.reference().clone(),
            parse_git_object(text.trim(), manifest.object_format())?,
        ));
    }
    refs.sort();
    Ok(refs)
}

fn reject_ambient_git_sources(git: &str) -> Result<(), GuestRunnerError> {
    for key in ["GIT_OBJECT_DIRECTORY", "GIT_ALTERNATE_OBJECT_DIRECTORIES"] {
        if std::env::var_os(key).is_some() {
            return Err(GuestRunnerError::AmbientGitEnvironment(key));
        }
    }

    let repo = Path::new(GUEST_REPLAY_REPOSITORY);
    for path in ["objects/info/alternates", "objects/info/http-alternates", "shallow"] {
        if repo.join(path).exists() {
            return Err(GuestRunnerError::ForbiddenGitState(path.to_owned()));
        }
    }

    let shallow_args = ["rev-parse".to_owned(), "--is-shallow-repository".to_owned()];
    let shallow = run_git_owned(git, &shallow_args, repo)?;
    require_success_owned(git, &shallow_args, &shallow)?;
    if trimmed_utf8(&shallow.stdout, "shallow check")? != "false" {
        return Err(GuestRunnerError::ForbiddenGitState("shallow repository".into()));
    }

    require_config_absent(git, repo, &["config", "--local", "--get", "extensions.partialClone"])?;
    require_config_absent(
        git,
        repo,
        &["config", "--local", "--get-regexp", r"^remote\..*\.promisor$"],
    )?;

    let remote_args = ["remote".to_owned()];
    let remotes = run_git_owned(git, &remote_args, repo)?;
    require_success_owned(git, &remote_args, &remotes)?;
    if !trimmed_utf8(&remotes.stdout, "remote list")?.is_empty() {
        return Err(GuestRunnerError::ConfiguredRemotePresent);
    }

    let pack_dir = repo.join("objects/pack");
    if pack_dir.exists() {
        for entry in fs::read_dir(pack_dir)? {
            if entry?.path().extension() == Some(OsStr::new("promisor")) {
                return Err(GuestRunnerError::PromisorPackPresent);
            }
        }
    }
    Ok(())
}

fn require_config_absent(
    git: &str,
    repo: &Path,
    args: &[&str],
) -> Result<(), GuestRunnerError> {
    let output = run_exact(git, args, repo)?;
    match output.status.code() {
        Some(1) if output.stdout.is_empty() => Ok(()),
        Some(0) => Err(GuestRunnerError::ForbiddenGitConfig(args.join(" "))),
        _ => Err(command_failed(git, args, &output)),
    }
}

fn run_strict_git_fsck(git: &str) -> Result<(), GuestRunnerError> {
    let policy = GitObjectValidationPolicyV1::strict();
    let mut args = policy.global_args().to_vec();
    args.extend_from_slice(policy.fsck_args());
    let output = run_exact(git, &args, Path::new(GUEST_REPLAY_REPOSITORY))?;
    require_success(git, &args, &output)
}

fn run_git_owned(git: &str, args: &[String], cwd: &Path) -> Result<Output, GuestRunnerError> {
    let refs = args.iter().map(String::as_str).collect::<Vec<_>>();
    run_exact(git, &refs, cwd)
}

fn run_exact(program: &str, args: &[&str], cwd: &Path) -> Result<Output, GuestRunnerError> {
    let mut command = Command::new(program);
    command.args(args).current_dir(cwd).env_clear();
    for entry in expected_environment() {
        command.env(entry.key(), entry.value());
    }
    command.output().map_err(|source| GuestRunnerError::Spawn {
        program: program.to_owned(),
        source,
    })
}

fn require_success(program: &str, args: &[&str], output: &Output) -> Result<(), GuestRunnerError> {
    if output.status.success() {
        Ok(())
    } else {
        Err(command_failed(program, args, output))
    }
}

fn require_success_owned(
    program: &str,
    args: &[String],
    output: &Output,
) -> Result<(), GuestRunnerError> {
    let refs = args.iter().map(String::as_str).collect::<Vec<_>>();
    require_success(program, &refs, output)
}

fn command_failed(program: &str, args: &[&str], output: &Output) -> GuestRunnerError {
    let mut stderr = String::from_utf8_lossy(&output.stderr).into_owned();
    stderr.truncate(stderr.len().min(MAX_COMMAND_DIAGNOSTIC));
    GuestRunnerError::CommandFailed {
        program: program.to_owned(),
        args: args.iter().map(|value| (*value).to_owned()).collect(),
        status: output.status.code().unwrap_or(-1),
        stderr,
    }
}

fn parse_git_object(
    text: &str,
    algorithm: GitObjectAlgorithm,
) -> Result<GitObjectId, GuestRunnerError> {
    let bytes = hex::decode(text).map_err(|_| GuestRunnerError::MalformedGitObjectId)?;
    Ok(GitObjectId::new(algorithm, bytes)?)
}

fn trimmed_utf8<'a>(bytes: &'a [u8], operation: &'static str) -> Result<&'a str, GuestRunnerError> {
    std::str::from_utf8(bytes)
        .map(str::trim)
        .map_err(|_| GuestRunnerError::NonUtf8CommandOutput(operation))
}

fn load_json<T: DeserializeOwned>(path: &Path) -> Result<T, GuestRunnerError> {
    let bytes = fs::read(path)?;
    Ok(serde_json::from_slice(&bytes)?)
}

fn write_json_atomic<T: Serialize>(path: &Path, value: &T) -> Result<(), GuestRunnerError> {
    let bytes = serde_json::to_vec(value)?;
    let parent = path.parent().ok_or(GuestRunnerError::InvalidOutputPath)?;
    let name = path
        .file_name()
        .and_then(OsStr::to_str)
        .ok_or(GuestRunnerError::InvalidOutputPath)?;
    let temporary = parent.join(format!(".{name}.tmp-{}", std::process::id()));
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&temporary)?;
    file.write_all(&bytes)?;
    file.sync_all()?;
    drop(file);
    fs::rename(&temporary, path)?;
    File::open(parent)?.sync_all()?;

    // The writer does not trust its own write syscall as evidence. Re-read the
    // exact file and require byte equality before returning success.
    let mut observed = Vec::new();
    File::open(path)?.read_to_end(&mut observed)?;
    if observed != bytes {
        return Err(GuestRunnerError::OutputRereadMismatch);
    }
    Ok(())
}

#[derive(Debug, Error)]
pub enum GuestRunnerError {
    #[error(transparent)]
    Adapter(#[from] AdapterError),
    #[error(transparent)]
    Bundle(#[from] OfflineBundleError),
    #[error(transparent)]
    PolicyInventory(#[from] PolicyInventoryError),
    #[error(transparent)]
    Trust(#[from] TrustProfileError),
    #[error(transparent)]
    GuestEnvelope(#[from] GuestEnvelopeError),
    #[error(transparent)]
    GuestPlan(#[from] GuestPlanError),
    #[error(transparent)]
    GuestToolMap(#[from] GuestToolMapError),
    #[error(transparent)]
    GuestTranscript(#[from] GuestTranscriptError),
    #[error(transparent)]
    IsolationPolicy(#[from] IsolationPolicyError),
    #[error(transparent)]
    IsolationCollector(#[from] IsolationCollectorError),
    #[error(transparent)]
    IsolationEvidence(#[from] IsolationEvidenceError),
    #[error(transparent)]
    Repository(#[from] RepositoryVerificationError),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error("invalid isolation-probe arguments")]
    InvalidProbeArguments,
    #[error("run challenge must contain exactly 32 bytes; got {0}")]
    InvalidRunChallengeSize(usize),
    #[error("run challenge bytes differ from the guest plan commitment")]
    RunChallengeMismatch,
    #[error("bundle manifest differs from the guest-plan semantic commitment")]
    ManifestCommitmentMismatch,
    #[error("bundle manifest policy state differs from the guest plan")]
    PolicyStateMismatch,
    #[error("Linux isolation policy differs from the guest plan")]
    IsolationPolicyMismatch,
    #[error("Nix closure differs from the guest plan")]
    ClosureMismatch,
    #[error("sandbox invocation differs from the guest plan")]
    SandboxInvocationMismatch,
    #[error("guest tool map names different plan/execution/closure subjects")]
    GuestToolMapMismatch,
    #[error("guest tool map is missing required tool {0}")]
    MissingGuestTool(String),
    #[error("local trust profile differs from guest plan")]
    TrustProfileMismatch,
    #[error("portable replay gittuf receipt differs from expected receipt")]
    ReplayReceiptMismatch,
    #[error("bundle artifact differs from manifest digest/size")]
    BundleArtifactMismatch,
    #[error("replay repository already exists")]
    ReplayRepositoryAlreadyExists,
    #[error("bundle advertised ref set differs from manifest")]
    BundleRefSetMismatch,
    #[error("imported replay ref set differs from manifest")]
    ImportedRefSetMismatch,
    #[error("ambient Git environment variable is present: {0}")]
    AmbientGitEnvironment(&'static str),
    #[error("forbidden Git replay state: {0}")]
    ForbiddenGitState(String),
    #[error("forbidden Git configuration exists: {0}")]
    ForbiddenGitConfig(String),
    #[error("configured Git remote exists in replay repository")]
    ConfiguredRemotePresent,
    #[error("promisor pack marker exists in replay repository")]
    PromisorPackPresent,
    #[error("malformed Git object ID")]
    MalformedGitObjectId,
    #[error("non-UTF-8 command output while performing {0}")]
    NonUtf8CommandOutput(&'static str),
    #[error("failed to spawn {program}: {source}")]
    Spawn {
        program: String,
        #[source]
        source: std::io::Error,
    },
    #[error("command failed: {program} {args:?}, status={status}, stderr={stderr}")]
    CommandFailed {
        program: String,
        args: Vec<String>,
        status: i32,
        stderr: String,
    },
    #[error("invalid guest output path")]
    InvalidOutputPath,
    #[error("atomically written guest output differed when re-read")]
    OutputRereadMismatch,
}
