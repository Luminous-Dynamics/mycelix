// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical Linux isolation policy for Mycelix Forge.
//!
//! FORGE-004D2A freezes one exact bubblewrap/Nix execution policy and command
//! construction. It does not claim that a host kernel actually enforced the
//! policy; live namespace/mount/network qualification is FORGE-004D2B.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};
use thiserror::Error;

const CLOSURE_DOMAIN_V1: &[u8] = b"mycelix-forge/nix-closure/v1\0";
const INVOCATION_DOMAIN_V1: &[u8] = b"mycelix-forge/verifier-invocation/v1\0";
const POLICY_DOMAIN_V1: &[u8] = b"mycelix-forge/linux-isolation-policy/v1\0";
const MAX_TEXT: usize = 4096;
const MAX_ARGS: usize = 4096;

pub const SANDBOX_HOME: &str = "/home/forge";
pub const SANDBOX_WORKDIR: &str = "/work";
pub const SANDBOX_TMP: &str = "/tmp";
pub const SANDBOX_HOSTNAME: &str = "forge-verifier";

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct NixClosureEntry {
    store_path: String,
    nar_hash: Digest,
}

impl NixClosureEntry {
    pub fn new(
        store_path: impl Into<String>,
        nar_hash: Digest,
    ) -> Result<Self, IsolationPolicyError> {
        let store_path = store_path.into();
        validate_store_root(&store_path)?;
        Ok(Self {
            store_path,
            nar_hash,
        })
    }

    pub fn store_path(&self) -> &str {
        &self.store_path
    }

    pub fn nar_hash(&self) -> &Digest {
        &self.nar_hash
    }
}

impl<'de> Deserialize<'de> for NixClosureEntry {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            store_path: String,
            nar_hash: Digest,
        }

        let wire = Wire::deserialize(deserializer)?;
        Self::new(wire.store_path, wire.nar_hash).map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct NixClosureManifest {
    entries: Vec<NixClosureEntry>,
}

impl NixClosureManifest {
    pub fn new(mut entries: Vec<NixClosureEntry>) -> Result<Self, IsolationPolicyError> {
        if entries.is_empty() {
            return Err(IsolationPolicyError::EmptyClosure);
        }
        entries.sort();
        for pair in entries.windows(2) {
            if pair[0].store_path == pair[1].store_path {
                return Err(IsolationPolicyError::DuplicateStorePath(
                    pair[0].store_path.clone(),
                ));
            }
        }
        Ok(Self { entries })
    }

    pub fn entries(&self) -> &[NixClosureEntry] {
        &self.entries
    }

    pub fn contains_executable(&self, executable: &str) -> bool {
        self.entries.iter().any(|entry| {
            executable
                .strip_prefix(&entry.store_path)
                .is_some_and(|suffix| suffix.starts_with('/') && suffix.len() > 1)
        })
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, IsolationPolicyError> {
        let mut out = Vec::new();
        out.extend_from_slice(CLOSURE_DOMAIN_V1);
        push_count(&mut out, self.entries.len(), "closure entries")?;
        for entry in &self.entries {
            push_string(&mut out, &entry.store_path, "store path")?;
            push_digest(&mut out, &entry.nar_hash)?;
        }
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, IsolationPolicyError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for NixClosureManifest {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            entries: Vec<NixClosureEntry>,
        }

        let wire = Wire::deserialize(deserializer)?;
        Self::new(wire.entries).map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct VerifierInvocation {
    executable: String,
    args: Vec<String>,
}

impl VerifierInvocation {
    pub fn new(
        executable: impl Into<String>,
        args: Vec<String>,
        closure: &NixClosureManifest,
    ) -> Result<Self, IsolationPolicyError> {
        let executable = executable.into();
        validate_store_executable(&executable)?;
        if !closure.contains_executable(&executable) {
            return Err(IsolationPolicyError::ExecutableOutsideClosure(executable));
        }
        if args.len() > MAX_ARGS {
            return Err(IsolationPolicyError::TooManyArguments(args.len()));
        }
        for arg in &args {
            if arg.len() > MAX_TEXT || arg.contains('\0') {
                return Err(IsolationPolicyError::InvalidText("argument"));
            }
        }
        Ok(Self { executable, args })
    }

    pub fn executable(&self) -> &str {
        &self.executable
    }

    pub fn args(&self) -> &[String] {
        &self.args
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, IsolationPolicyError> {
        let mut out = Vec::new();
        out.extend_from_slice(INVOCATION_DOMAIN_V1);
        push_string(&mut out, &self.executable, "executable")?;
        push_count(&mut out, self.args.len(), "arguments")?;
        for arg in &self.args {
            push_string(&mut out, arg, "argument")?;
        }
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, IsolationPolicyError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct ArtifactMount {
    role: String,
    destination: String,
    digest: Digest,
    size: u64,
}

impl ArtifactMount {
    pub fn new(
        role: impl Into<String>,
        destination: impl Into<String>,
        digest: Digest,
        size: u64,
    ) -> Result<Self, IsolationPolicyError> {
        let role = role.into();
        let destination = destination.into();
        validate_text("artifact role", &role)?;
        validate_sandbox_mount(&destination)?;
        if size == 0 {
            return Err(IsolationPolicyError::EmptyArtifact(role));
        }
        Ok(Self {
            role,
            destination,
            digest,
            size,
        })
    }

    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn destination(&self) -> &str {
        &self.destination
    }

    pub fn digest(&self) -> &Digest {
        &self.digest
    }

    pub const fn size(&self) -> u64 {
        self.size
    }
}

impl<'de> Deserialize<'de> for ArtifactMount {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            role: String,
            destination: String,
            digest: Digest,
            size: u64,
        }

        let wire = Wire::deserialize(deserializer)?;
        Self::new(wire.role, wire.destination, wire.digest, wire.size).map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct LinuxIsolationPolicyV1 {
    nix_closure: Digest,
    invocation: Digest,
    artifact_mounts: Vec<ArtifactMount>,
    unshare_user: bool,
    unshare_ipc: bool,
    unshare_pid: bool,
    unshare_net: bool,
    unshare_uts: bool,
    disable_nested_userns: bool,
    new_session: bool,
    die_with_parent: bool,
    drop_all_capabilities: bool,
    clear_environment: bool,
    private_proc: bool,
    minimal_dev: bool,
    host_home_hidden: bool,
}

impl LinuxIsolationPolicyV1 {
    pub fn strict(
        closure: &NixClosureManifest,
        invocation: &VerifierInvocation,
        mut artifact_mounts: Vec<ArtifactMount>,
    ) -> Result<Self, IsolationPolicyError> {
        artifact_mounts.sort();
        validate_unique_mounts(&artifact_mounts)?;
        Ok(Self {
            nix_closure: closure.digest(DigestAlgorithm::Sha256)?,
            invocation: invocation.digest(DigestAlgorithm::Sha256)?,
            artifact_mounts,
            unshare_user: true,
            unshare_ipc: true,
            unshare_pid: true,
            unshare_net: true,
            unshare_uts: true,
            disable_nested_userns: true,
            new_session: true,
            die_with_parent: true,
            drop_all_capabilities: true,
            clear_environment: true,
            private_proc: true,
            minimal_dev: true,
            host_home_hidden: true,
        })
    }

    pub fn artifact_mounts(&self) -> &[ArtifactMount] {
        &self.artifact_mounts
    }

    pub fn nix_closure(&self) -> &Digest {
        &self.nix_closure
    }

    pub fn invocation(&self) -> &Digest {
        &self.invocation
    }

    pub fn validate_dependencies(
        &self,
        closure: &NixClosureManifest,
        invocation: &VerifierInvocation,
    ) -> Result<(), IsolationPolicyError> {
        if closure.digest(self.nix_closure.algorithm())? != self.nix_closure {
            return Err(IsolationPolicyError::ClosureDigestMismatch);
        }
        if invocation.digest(self.invocation.algorithm())? != self.invocation {
            return Err(IsolationPolicyError::InvocationDigestMismatch);
        }
        self.validate_strict_flags()
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, IsolationPolicyError> {
        self.validate_strict_flags()?;
        validate_mount_set_canonical(&self.artifact_mounts)?;
        let mut out = Vec::new();
        out.extend_from_slice(POLICY_DOMAIN_V1);
        push_digest(&mut out, &self.nix_closure)?;
        push_digest(&mut out, &self.invocation)?;
        push_count(&mut out, self.artifact_mounts.len(), "artifact mounts")?;
        for mount in &self.artifact_mounts {
            push_string(&mut out, &mount.role, "artifact role")?;
            push_string(&mut out, &mount.destination, "artifact destination")?;
            push_digest(&mut out, &mount.digest)?;
            out.extend_from_slice(&mount.size.to_be_bytes());
        }
        for flag in self.flags() {
            out.push(u8::from(flag));
        }
        push_string(&mut out, SANDBOX_HOSTNAME, "hostname")?;
        push_string(&mut out, SANDBOX_HOME, "home")?;
        push_string(&mut out, SANDBOX_WORKDIR, "workdir")?;
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, IsolationPolicyError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }

    fn flags(&self) -> [bool; 13] {
        [
            self.unshare_user,
            self.unshare_ipc,
            self.unshare_pid,
            self.unshare_net,
            self.unshare_uts,
            self.disable_nested_userns,
            self.new_session,
            self.die_with_parent,
            self.drop_all_capabilities,
            self.clear_environment,
            self.private_proc,
            self.minimal_dev,
            self.host_home_hidden,
        ]
    }

    fn validate_strict_flags(&self) -> Result<(), IsolationPolicyError> {
        if self.flags().into_iter().all(|flag| flag) {
            Ok(())
        } else {
            Err(IsolationPolicyError::WeakenedV1Policy)
        }
    }
}

impl<'de> Deserialize<'de> for LinuxIsolationPolicyV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            nix_closure: Digest,
            invocation: Digest,
            artifact_mounts: Vec<ArtifactMount>,
            unshare_user: bool,
            unshare_ipc: bool,
            unshare_pid: bool,
            unshare_net: bool,
            unshare_uts: bool,
            disable_nested_userns: bool,
            new_session: bool,
            die_with_parent: bool,
            drop_all_capabilities: bool,
            clear_environment: bool,
            private_proc: bool,
            minimal_dev: bool,
            host_home_hidden: bool,
        }

        let wire = Wire::deserialize(deserializer)?;
        let policy = Self {
            nix_closure: wire.nix_closure,
            invocation: wire.invocation,
            artifact_mounts: wire.artifact_mounts,
            unshare_user: wire.unshare_user,
            unshare_ipc: wire.unshare_ipc,
            unshare_pid: wire.unshare_pid,
            unshare_net: wire.unshare_net,
            unshare_uts: wire.unshare_uts,
            disable_nested_userns: wire.disable_nested_userns,
            new_session: wire.new_session,
            die_with_parent: wire.die_with_parent,
            drop_all_capabilities: wire.drop_all_capabilities,
            clear_environment: wire.clear_environment,
            private_proc: wire.private_proc,
            minimal_dev: wire.minimal_dev,
            host_home_hidden: wire.host_home_hidden,
        };
        validate_mount_set_canonical(&policy.artifact_mounts).map_err(D::Error::custom)?;
        policy.validate_strict_flags().map_err(D::Error::custom)?;
        Ok(policy)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RuntimeArtifact {
    pub role: String,
    pub source: PathBuf,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BubblewrapCommand {
    pub program: PathBuf,
    pub args: Vec<String>,
}

/// Build the exact v1 bubblewrap command without a shell.
///
/// FORGE-004D2B must hash each runtime artifact, verify each NAR hash, bind the
/// bubblewrap executable itself as an exact tool artifact, and collect live
/// isolation evidence before executing this argv.
pub fn build_bubblewrap_command(
    bubblewrap_program: impl Into<PathBuf>,
    policy: &LinuxIsolationPolicyV1,
    closure: &NixClosureManifest,
    invocation: &VerifierInvocation,
    runtime_artifacts: &[RuntimeArtifact],
) -> Result<BubblewrapCommand, IsolationPolicyError> {
    policy.validate_dependencies(closure, invocation)?;

    let runtime: BTreeMap<&str, &PathBuf> = runtime_artifacts
        .iter()
        .map(|artifact| (artifact.role.as_str(), &artifact.source))
        .collect();
    if runtime.len() != runtime_artifacts.len() {
        return Err(IsolationPolicyError::DuplicateRuntimeRole);
    }
    for mount in &policy.artifact_mounts {
        if !runtime.contains_key(mount.role()) {
            return Err(IsolationPolicyError::MissingRuntimeArtifact(
                mount.role().to_owned(),
            ));
        }
    }
    if runtime.len() != policy.artifact_mounts.len() {
        return Err(IsolationPolicyError::UnexpectedRuntimeArtifact);
    }

    let mut args = vec![
        "--unshare-user".into(),
        "--unshare-ipc".into(),
        "--unshare-pid".into(),
        "--unshare-net".into(),
        "--unshare-uts".into(),
        "--disable-userns".into(),
        "--new-session".into(),
        "--die-with-parent".into(),
        "--cap-drop".into(),
        "ALL".into(),
        "--clearenv".into(),
        "--hostname".into(),
        SANDBOX_HOSTNAME.into(),
        "--proc".into(),
        "/proc".into(),
        "--dev".into(),
        "/dev".into(),
        "--tmpfs".into(),
        SANDBOX_TMP.into(),
        "--dir".into(),
        "/nix".into(),
        "--dir".into(),
        "/nix/store".into(),
        "--dir".into(),
        "/home".into(),
        "--dir".into(),
        SANDBOX_HOME.into(),
        "--dir".into(),
        SANDBOX_WORKDIR.into(),
    ];

    // Mount only the exact Nix closure. Never expose the host store wholesale.
    for entry in closure.entries() {
        args.extend([
            "--ro-bind".into(),
            entry.store_path().into(),
            entry.store_path().into(),
        ]);
    }

    for mount in &policy.artifact_mounts {
        let source = runtime
            .get(mount.role())
            .expect("runtime role existence checked above")
            .to_string_lossy()
            .into_owned();
        args.extend([
            "--ro-bind".into(),
            source,
            mount.destination().into(),
        ]);
    }

    args.extend([
        "--setenv".into(),
        "HOME".into(),
        SANDBOX_HOME.into(),
        "--setenv".into(),
        "TMPDIR".into(),
        SANDBOX_TMP.into(),
        "--setenv".into(),
        "PWD".into(),
        SANDBOX_WORKDIR.into(),
        "--setenv".into(),
        "LANG".into(),
        "C".into(),
        "--setenv".into(),
        "LC_ALL".into(),
        "C".into(),
        "--setenv".into(),
        "GIT_NO_LAZY_FETCH".into(),
        "1".into(),
        "--setenv".into(),
        "GIT_TERMINAL_PROMPT".into(),
        "0".into(),
        "--setenv".into(),
        "GIT_CONFIG_NOSYSTEM".into(),
        "1".into(),
        "--setenv".into(),
        "GIT_CONFIG_GLOBAL".into(),
        "/dev/null".into(),
        "--setenv".into(),
        "GITTUF_DEV".into(),
        "0".into(),
        "--setenv".into(),
        "GITTUF_DEBUG".into(),
        "0".into(),
        "--chdir".into(),
        SANDBOX_WORKDIR.into(),
        "--".into(),
        invocation.executable().into(),
    ]);
    args.extend(invocation.args().iter().cloned());

    Ok(BubblewrapCommand {
        program: bubblewrap_program.into(),
        args,
    })
}

fn validate_unique_mounts(mounts: &[ArtifactMount]) -> Result<(), IsolationPolicyError> {
    let mut roles = BTreeSet::new();
    let mut destinations = BTreeSet::new();
    for mount in mounts {
        if !roles.insert(mount.role.clone()) {
            return Err(IsolationPolicyError::DuplicateArtifactRole(
                mount.role.clone(),
            ));
        }
        if !destinations.insert(mount.destination.clone()) {
            return Err(IsolationPolicyError::DuplicateDestination(
                mount.destination.clone(),
            ));
        }
    }
    Ok(())
}

fn validate_mount_set_canonical(mounts: &[ArtifactMount]) -> Result<(), IsolationPolicyError> {
    validate_unique_mounts(mounts)?;
    let mut sorted = mounts.to_vec();
    sorted.sort();
    if sorted != mounts {
        Err(IsolationPolicyError::MountsNotCanonical)
    } else {
        Ok(())
    }
}

fn validate_store_root(value: &str) -> Result<(), IsolationPolicyError> {
    validate_text("Nix store path", value)?;
    let Some(suffix) = value.strip_prefix("/nix/store/") else {
        return Err(IsolationPolicyError::InvalidStorePath(value.to_owned()));
    };
    if suffix.is_empty() || suffix.contains('/') || suffix == "." || suffix == ".." {
        return Err(IsolationPolicyError::InvalidStorePath(value.to_owned()));
    }
    Ok(())
}

fn validate_store_executable(value: &str) -> Result<(), IsolationPolicyError> {
    validate_text("executable", value)?;
    if !value.starts_with("/nix/store/")
        || value.ends_with('/')
        || value.contains("//")
        || value.contains("/../")
        || value.contains("/./")
    {
        return Err(IsolationPolicyError::InvalidExecutablePath(
            value.to_owned(),
        ));
    }
    Ok(())
}

fn validate_sandbox_mount(value: &str) -> Result<(), IsolationPolicyError> {
    validate_text("sandbox mount", value)?;
    let valid_root = value.starts_with("/inputs/") || value.starts_with("/trust/");
    if !valid_root
        || value.ends_with('/')
        || value.contains("//")
        || value.contains("/../")
        || value.contains("/./")
    {
        return Err(IsolationPolicyError::InvalidMountDestination(
            value.to_owned(),
        ));
    }
    Ok(())
}

fn validate_text(field: &'static str, value: &str) -> Result<(), IsolationPolicyError> {
    if value.is_empty() || value.len() > MAX_TEXT || value.contains('\0') {
        Err(IsolationPolicyError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn push_count(
    out: &mut Vec<u8>,
    count: usize,
    field: &'static str,
) -> Result<(), IsolationPolicyError> {
    let count = u16::try_from(count)
        .map_err(|_| IsolationPolicyError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&count.to_be_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
    field: &'static str,
) -> Result<(), IsolationPolicyError> {
    let len = u16::try_from(value.len())
        .map_err(|_| IsolationPolicyError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), IsolationPolicyError> {
    push_string(out, digest.algorithm().id(), "digest algorithm")?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| IsolationPolicyError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum IsolationPolicyError {
    #[error("invalid text field: {0}")]
    InvalidText(&'static str),
    #[error("Nix closure may not be empty")]
    EmptyClosure,
    #[error("invalid Nix store root: {0}")]
    InvalidStorePath(String),
    #[error("duplicate Nix store path: {0}")]
    DuplicateStorePath(String),
    #[error("invalid verifier executable path: {0}")]
    InvalidExecutablePath(String),
    #[error("verifier executable is outside the committed Nix closure: {0}")]
    ExecutableOutsideClosure(String),
    #[error("too many verifier arguments: {0}")]
    TooManyArguments(usize),
    #[error("invalid sandbox mount destination: {0}")]
    InvalidMountDestination(String),
    #[error("artifact {0} may not be empty")]
    EmptyArtifact(String),
    #[error("duplicate artifact role: {0}")]
    DuplicateArtifactRole(String),
    #[error("duplicate sandbox destination: {0}")]
    DuplicateDestination(String),
    #[error("artifact mounts are not in canonical order")]
    MountsNotCanonical,
    #[error("v1 isolation policy was weakened")]
    WeakenedV1Policy,
    #[error("Nix closure commitment mismatch")]
    ClosureDigestMismatch,
    #[error("verifier invocation commitment mismatch")]
    InvocationDigestMismatch,
    #[error("duplicate runtime artifact role")]
    DuplicateRuntimeRole,
    #[error("missing runtime artifact for role {0}")]
    MissingRuntimeArtifact(String),
    #[error("runtime provided an artifact not committed by policy")]
    UnexpectedRuntimeArtifact,
    #[error("canonical field is too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn closure() -> NixClosureManifest {
        NixClosureManifest::new(vec![
            NixClosureEntry::new("/nix/store/aaaa-git-2.52", digest(1)).unwrap(),
            NixClosureEntry::new("/nix/store/bbbb-gittuf-0.16.0", digest(2)).unwrap(),
            NixClosureEntry::new("/nix/store/cccc-glibc", digest(3)).unwrap(),
        ])
        .unwrap()
    }

    fn invocation(closure: &NixClosureManifest) -> VerifierInvocation {
        VerifierInvocation::new(
            "/nix/store/bbbb-gittuf-0.16.0/bin/gittuf",
            vec!["verify-ref".into(), "refs/heads/main".into()],
            closure,
        )
        .unwrap()
    }

    fn mounts() -> Vec<ArtifactMount> {
        vec![
            ArtifactMount::new("manifest", "/inputs/manifest.json", digest(4), 200).unwrap(),
            ArtifactMount::new(
                "repository-bundle",
                "/inputs/repository.bundle",
                digest(5),
                500,
            )
            .unwrap(),
        ]
    }

    #[test]
    fn closure_order_is_canonical() {
        let a = closure();
        let b = NixClosureManifest::new(a.entries().iter().cloned().rev().collect()).unwrap();
        assert_eq!(
            a.digest(DigestAlgorithm::Sha256).unwrap(),
            b.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn store_entries_must_be_roots_not_subpaths() {
        assert!(NixClosureEntry::new("/nix/store/aaaa-pkg/bin/tool", digest(1)).is_err());
    }

    #[test]
    fn executable_must_live_in_committed_closure_without_traversal() {
        let closure = closure();
        assert!(VerifierInvocation::new("/usr/bin/git", vec!["status".into()], &closure).is_err());
        assert!(VerifierInvocation::new(
            "/nix/store/bbbb-gittuf-0.16.0/../evil",
            vec![],
            &closure,
        )
        .is_err());
    }

    #[test]
    fn duplicate_destination_is_rejected_even_with_different_roles() {
        let closure = closure();
        let invocation = invocation(&closure);
        let mounts = vec![
            ArtifactMount::new("a", "/inputs/same", digest(1), 1).unwrap(),
            ArtifactMount::new("z", "/inputs/same", digest(2), 1).unwrap(),
        ];
        assert!(matches!(
            LinuxIsolationPolicyV1::strict(&closure, &invocation, mounts),
            Err(IsolationPolicyError::DuplicateDestination(_))
        ));
    }

    #[test]
    fn weakened_serialized_policy_is_rejected() {
        let closure = closure();
        let invocation = invocation(&closure);
        let policy = LinuxIsolationPolicyV1::strict(&closure, &invocation, mounts()).unwrap();
        let mut value = serde_json::to_value(policy).unwrap();
        value["unshare_net"] = serde_json::json!(false);
        assert!(serde_json::from_value::<LinuxIsolationPolicyV1>(value).is_err());
    }

    #[test]
    fn command_mounts_exact_closure_not_whole_store() {
        let closure = closure();
        let invocation = invocation(&closure);
        let policy = LinuxIsolationPolicyV1::strict(&closure, &invocation, mounts()).unwrap();
        let runtime = vec![
            RuntimeArtifact {
                role: "manifest".into(),
                source: "/host/evidence/manifest.json".into(),
            },
            RuntimeArtifact {
                role: "repository-bundle".into(),
                source: "/host/evidence/repository.bundle".into(),
            },
        ];
        let command = build_bubblewrap_command(
            "/nix/store/dddd-bubblewrap/bin/bwrap",
            &policy,
            &closure,
            &invocation,
            &runtime,
        )
        .unwrap();
        assert!(command.args.iter().any(|arg| arg == "--unshare-net"));
        assert!(command.args.iter().any(|arg| arg == "--clearenv"));
        assert!(command
            .args
            .windows(2)
            .any(|w| w[0] == "--cap-drop" && w[1] == "ALL"));
        assert!(!command.args.windows(3).any(|w| {
            w[0] == "--ro-bind" && w[1] == "/nix/store" && w[2] == "/nix/store"
        }));
        for entry in closure.entries() {
            assert!(command.args.windows(3).any(|w| {
                w[0] == "--ro-bind"
                    && w[1] == entry.store_path()
                    && w[2] == entry.store_path()
            }));
        }
    }

    #[test]
    fn runtime_artifact_set_must_match_policy_exactly() {
        let closure = closure();
        let invocation = invocation(&closure);
        let policy = LinuxIsolationPolicyV1::strict(&closure, &invocation, mounts()).unwrap();
        let runtime = vec![RuntimeArtifact {
            role: "manifest".into(),
            source: "/a".into(),
        }];
        assert!(matches!(
            build_bubblewrap_command("bwrap", &policy, &closure, &invocation, &runtime),
            Err(IsolationPolicyError::MissingRuntimeArtifact(_))
        ));
    }

    #[test]
    fn host_paths_do_not_change_policy_commitment() {
        let closure = closure();
        let invocation = invocation(&closure);
        let policy = LinuxIsolationPolicyV1::strict(&closure, &invocation, mounts()).unwrap();
        let before = policy.digest(DigestAlgorithm::Sha256).unwrap();
        let _runtime_a = RuntimeArtifact {
            role: "manifest".into(),
            source: "/srv/a/manifest.json".into(),
        };
        let _runtime_b = RuntimeArtifact {
            role: "manifest".into(),
            source: "/mnt/b/manifest.json".into(),
        };
        assert_eq!(before, policy.digest(DigestAlgorithm::Sha256).unwrap());
    }
}
