// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use crate::IsolationCollectorError;
use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_linux_isolation::{LinuxIsolationPolicyV1, SANDBOX_TMP, SANDBOX_WORKDIR};
use mycelix_forge_linux_isolation_evidence::{
    ArtifactObservation, CapabilitySnapshot, EnvironmentEntry, InsideIsolationEvidence,
    NamespaceSnapshot,
};
use sha2::{Digest as ShaDigest, Sha256};
use std::{
    fs::{self, File, OpenOptions},
    io::Read,
    net::{IpAddr, Ipv4Addr, SocketAddr, TcpStream},
    path::{Path, PathBuf},
    time::Duration,
};

const CONNECT_TIMEOUT: Duration = Duration::from_millis(200);

/// Collect the active observation channel from inside an already-created
/// FORGE-004D2A sandbox.
///
/// Nested-user-namespace creation is probed last because a surprising success
/// mutates the current process namespace. Callers must terminate the probe
/// process after this function returns; the real verifier must run separately.
pub fn collect_inside_evidence(
    policy: &LinuxIsolationPolicyV1,
) -> Result<InsideIsolationEvidence, IsolationCollectorError> {
    let namespaces = current_namespaces()?;
    let status = fs::read_to_string("/proc/self/status")?;
    let capabilities = parse_capabilities(&status)?;
    let hostname = fs::read_to_string("/proc/sys/kernel/hostname")?
        .trim()
        .to_owned();

    let environment = collect_environment()?;
    let visible_store_roots = read_directory_names(Path::new("/nix/store"))?;
    let artifacts = collect_artifacts(policy)?;
    let workdir_writable = probe_writable_directory(Path::new(SANDBOX_WORKDIR))?;
    let tmp_writable = probe_writable_directory(Path::new(SANDBOX_TMP))?;
    let home_entries = read_directory_names(Path::new("/home"))?;
    let sys_present = Path::new("/sys").exists();
    let run_present = Path::new("/run").exists();

    let mountinfo = fs::read("/proc/self/mountinfo")?;
    let mountinfo_digest = sha256(&mountinfo);
    let route_table = fs::read("/proc/net/route")?;
    let route_table_digest = sha256(&route_table);
    let non_loopback_route_present = has_non_loopback_route(&route_table)?;

    let probe_address = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(1, 1, 1, 1)), 443);
    let outbound_connect_succeeded =
        TcpStream::connect_timeout(&probe_address, CONNECT_TIMEOUT).is_ok();

    // Destructive probe intentionally last. A success is a qualification
    // failure and this process must not be re-used as a verifier afterward.
    let nested_userns_created = unsafe { libc::unshare(libc::CLONE_NEWUSER) } == 0;

    Ok(InsideIsolationEvidence::new(
        policy.digest(DigestAlgorithm::Sha256)?,
        namespaces,
        capabilities,
        hostname,
        environment,
        visible_store_roots,
        artifacts,
        workdir_writable,
        tmp_writable,
        home_entries,
        sys_present,
        run_present,
        mountinfo_digest,
        route_table_digest,
        non_loopback_route_present,
        outbound_connect_succeeded,
        nested_userns_created,
    )?)
}

fn current_namespaces() -> Result<NamespaceSnapshot, IsolationCollectorError> {
    namespace_snapshot(Path::new("/proc/self"))
}

pub(crate) fn namespace_snapshot(
    root: &Path,
) -> Result<NamespaceSnapshot, IsolationCollectorError> {
    Ok(NamespaceSnapshot {
        user: read_namespace(root, "user")?,
        mount: read_namespace(root, "mnt")?,
        pid: read_namespace(root, "pid")?,
        ipc: read_namespace(root, "ipc")?,
        net: read_namespace(root, "net")?,
        uts: read_namespace(root, "uts")?,
    })
}

fn read_namespace(root: &Path, name: &str) -> Result<String, IsolationCollectorError> {
    let link = fs::read_link(root.join("ns").join(name))?;
    link.to_str()
        .map(ToOwned::to_owned)
        .ok_or(IsolationCollectorError::NonUtf8Path)
}

fn parse_capabilities(status: &str) -> Result<CapabilitySnapshot, IsolationCollectorError> {
    Ok(CapabilitySnapshot {
        inheritable: status_hex(status, "CapInh")?,
        permitted: status_hex(status, "CapPrm")?,
        effective: status_hex(status, "CapEff")?,
        bounding: status_hex(status, "CapBnd")?,
        ambient: status_hex(status, "CapAmb")?,
    })
}

fn status_hex(status: &str, key: &'static str) -> Result<u64, IsolationCollectorError> {
    let prefix = format!("{key}:\t");
    let value = status
        .lines()
        .find_map(|line| line.strip_prefix(&prefix))
        .ok_or(IsolationCollectorError::MissingProcStatusField(key))?;
    u64::from_str_radix(value.trim(), 16)
        .map_err(|_| IsolationCollectorError::MalformedProcStatusField(key))
}

fn collect_environment() -> Result<Vec<EnvironmentEntry>, IsolationCollectorError> {
    let mut values = Vec::new();
    for (key, value) in std::env::vars_os() {
        let key = key
            .into_string()
            .map_err(|_| IsolationCollectorError::NonUtf8Environment)?;
        let value = value
            .into_string()
            .map_err(|_| IsolationCollectorError::NonUtf8Environment)?;
        values.push(EnvironmentEntry::new(key, value)?);
    }
    Ok(values)
}

fn collect_artifacts(
    policy: &LinuxIsolationPolicyV1,
) -> Result<Vec<ArtifactObservation>, IsolationCollectorError> {
    let mut observations = Vec::with_capacity(policy.artifact_mounts().len());
    for mount in policy.artifact_mounts() {
        let path = Path::new(mount.destination());
        let metadata = fs::metadata(path)?;
        if !metadata.is_file() {
            return Err(IsolationCollectorError::ArtifactNotRegularFile(
                path.to_path_buf(),
            ));
        }
        let (digest, size) = hash_file(path, mount.digest().algorithm())?;
        let writable = OpenOptions::new().write(true).open(path).is_ok();
        observations.push(ArtifactObservation::new(
            mount.role(),
            mount.destination(),
            digest,
            size,
            true,
            writable,
        )?);
    }
    Ok(observations)
}

fn hash_file(
    path: &Path,
    algorithm: DigestAlgorithm,
) -> Result<(Digest, u64), IsolationCollectorError> {
    let mut file = File::open(path)?;
    let mut buffer = [0_u8; 64 * 1024];
    let mut size = 0_u64;

    match algorithm {
        DigestAlgorithm::Sha256 => {
            let mut hasher = Sha256::new();
            loop {
                let read = file.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                hasher.update(&buffer[..read]);
                size = size
                    .checked_add(read as u64)
                    .ok_or(IsolationCollectorError::ArtifactTooLarge)?;
            }
            Ok((
                Digest::new(DigestAlgorithm::Sha256, hasher.finalize().to_vec())?,
                size,
            ))
        }
        DigestAlgorithm::Blake3_256 => {
            let mut hasher = blake3::Hasher::new();
            loop {
                let read = file.read(&mut buffer)?;
                if read == 0 {
                    break;
                }
                hasher.update(&buffer[..read]);
                size = size
                    .checked_add(read as u64)
                    .ok_or(IsolationCollectorError::ArtifactTooLarge)?;
            }
            Ok((
                Digest::new(
                    DigestAlgorithm::Blake3_256,
                    hasher.finalize().as_bytes().to_vec(),
                )?,
                size,
            ))
        }
    }
}

fn read_directory_names(path: &Path) -> Result<Vec<String>, IsolationCollectorError> {
    let mut values = Vec::new();
    for entry in fs::read_dir(path)? {
        let name = entry?.file_name();
        values.push(
            name.to_str()
                .map(ToOwned::to_owned)
                .ok_or(IsolationCollectorError::NonUtf8Path)?,
        );
    }
    Ok(values)
}

fn probe_writable_directory(path: &Path) -> Result<bool, IsolationCollectorError> {
    let candidate: PathBuf = path.join(format!(".forge-isolation-probe-{}", std::process::id()));
    match OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&candidate)
    {
        Ok(_) => {
            fs::remove_file(candidate)?;
            Ok(true)
        }
        Err(_) => Ok(false),
    }
}

fn has_non_loopback_route(bytes: &[u8]) -> Result<bool, IsolationCollectorError> {
    let text =
        std::str::from_utf8(bytes).map_err(|_| IsolationCollectorError::NonUtf8RouteTable)?;
    for (index, line) in text.lines().enumerate() {
        if index == 0 || line.trim().is_empty() {
            continue;
        }
        let interface = line
            .split_whitespace()
            .next()
            .ok_or(IsolationCollectorError::MalformedRouteTable)?;
        if interface != "lo" {
            return Ok(true);
        }
    }
    Ok(false)
}

fn sha256(bytes: &[u8]) -> Digest {
    Digest::new(DigestAlgorithm::Sha256, Sha256::digest(bytes).to_vec())
        .expect("SHA-256 has the Forge protocol digest length")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capability_parser_accepts_zero_linux_masks() {
        let status = "CapInh:\t0000000000000000\nCapPrm:\t0000000000000000\nCapEff:\t0000000000000000\nCapBnd:\t0000000000000000\nCapAmb:\t0000000000000000\n";
        assert!(parse_capabilities(status).unwrap().all_zero());
    }

    #[test]
    fn route_parser_rejects_non_loopback_interface() {
        let table = b"Iface Destination Gateway Flags RefCnt Use Metric Mask MTU Window IRTT\neth0 00000000 0100007F 0003 0 0 0 00000000 0 0 0\n";
        assert!(has_non_loopback_route(table).unwrap());
    }

    #[test]
    fn route_parser_accepts_loopback_only() {
        let table = b"Iface Destination Gateway Flags RefCnt Use Metric Mask MTU Window IRTT\nlo 0000007F 00000000 0001 0 0 0 000000FF 0 0 0\n";
        assert!(!has_non_loopback_route(table).unwrap());
    }
}
