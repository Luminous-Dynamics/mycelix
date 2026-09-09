// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Linux host theorem for isolating the Holochain Admin endpoint behind one
//! process-local Mycelix broker boundary.
//!
//! The existing conductor-process fence proves which process owns one pinned
//! loopback Admin listener. Loopback alone is not an isolation boundary: every
//! ordinary process in the same network namespace can connect to it. This crate
//! therefore adds a stricter deployment theorem:
//!
//! - the broker is the current process and is pinned by UID + executable bytes;
//! - broker and conductor occupy one non-host Linux network namespace;
//! - that namespace exposes only the loopback interface; and
//! - the namespace process membership is exactly broker + pinned conductor at
//!   both sides of one caller-controlled interval.
//!
//! The positive result is historical interval evidence. It does not itself make
//! coordinator updates acquire a mutex and does not grant effect authority.
//! Root/kernel compromise, privileged `setns`, compromise of the pinned broker or
//! conductor, and transient privileged namespace entry remain outside v0.1.

#[cfg(not(target_os = "linux"))]
compile_error!("coordinator Admin isolation v0.1 requires Linux");

use mycelix_authority_coordinator_conductor_process::{
    ConductorProcessError, ConductorProcessFenceGuard, QualifiedConductorProcessFence,
    TrustedConductorProcessStore,
};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::fs::{self, File};
use std::io::{self, Read};
use std::net::SocketAddr;
use std::os::unix::fs::MetadataExt;
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

pub const PROTOCOL_VERSION: &str = "mycelix-authority-coordinator-admin-isolation-v0.1";
pub const BROKER_BINDING_PROFILE: &str =
    "mycelix-authority-coordinator-admin-broker-binding-v1-blake3-framed";
pub const BROKER_SNAPSHOT_PROFILE: &str =
    "mycelix-authority-coordinator-admin-broker-snapshot-v1-blake3-framed";
pub const ISOLATION_PROFILE: &str =
    "mycelix-authority-coordinator-admin-isolation-v1-blake3-framed";
pub const EXECUTABLE_PROFILE: &str = "mycelix-authority-coordinator-admin-broker-executable-v1-blake3-bytes";

const DOMAIN_BINDING: &[u8] = b"mycelix/authority/coordinator-admin-broker-binding/v1";
const DOMAIN_SNAPSHOT: &[u8] = b"mycelix/authority/coordinator-admin-broker-snapshot/v1";
const DOMAIN_ISOLATION: &[u8] = b"mycelix/authority/coordinator-admin-isolation/v1";
const MAX_TEXT_BYTES: usize = 4096;
const MAX_EXECUTABLE_BYTES: u64 = 1024 * 1024 * 1024;
const MAX_NAMESPACE_MEMBERS: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct NamespaceIdentity {
    pub device: u64,
    pub inode: u64,
}

impl NamespaceIdentity {
    fn validate(self) -> Result<(), AdminIsolationError> {
        if self.device == 0 || self.inode == 0 {
            Err(AdminIsolationError::InvalidNamespaceIdentity)
        } else {
            Ok(())
        }
    }
}

/// Out-of-band deployment pin for the broker process that owns final response
/// admission. This is provisioning data, not a positive live theorem.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BrokerProcessBinding {
    pub protocol_version: String,
    pub binding_id: String,
    pub admin_endpoint: SocketAddr,
    pub expected_broker_uid: u32,
    pub broker_executable_path: String,
    pub broker_executable_digest: [u8; 32],
    pub broker_executable_profile: String,
    pub conductor_binding_digest: [u8; 32],
    pub binding_ref: String,
}

impl BrokerProcessBinding {
    pub fn validate(&self) -> Result<(), AdminIsolationError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(AdminIsolationError::WrongProtocol);
        }
        validate_text(&self.binding_id, "broker binding id")?;
        if self.admin_endpoint.port() == 0 || !self.admin_endpoint.ip().is_loopback() {
            return Err(AdminIsolationError::InvalidAdminEndpoint);
        }
        validate_text(&self.broker_executable_path, "broker executable path")?;
        if !Path::new(&self.broker_executable_path).is_absolute() {
            return Err(AdminIsolationError::BrokerExecutablePathNotAbsolute);
        }
        validate_digest(&self.broker_executable_digest, "broker executable digest")?;
        if self.broker_executable_profile != EXECUTABLE_PROFILE {
            return Err(AdminIsolationError::WrongBrokerExecutableProfile);
        }
        validate_digest(&self.conductor_binding_digest, "conductor binding digest")?;
        validate_text(&self.binding_ref, "broker binding ref")?;
        Ok(())
    }

    pub fn binding_digest(&self) -> Result<[u8; 32], AdminIsolationError> {
        self.validate()?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_BINDING);
        frame(&mut h, PROTOCOL_VERSION.as_bytes());
        frame(&mut h, BROKER_BINDING_PROFILE.as_bytes());
        frame(&mut h, self.binding_id.as_bytes());
        frame(&mut h, self.admin_endpoint.to_string().as_bytes());
        frame(&mut h, &self.expected_broker_uid.to_le_bytes());
        frame(&mut h, self.broker_executable_path.as_bytes());
        frame(&mut h, &self.broker_executable_digest);
        frame(&mut h, self.broker_executable_profile.as_bytes());
        frame(&mut h, &self.conductor_binding_digest);
        frame(&mut h, self.binding_ref.as_bytes());
        Ok(*h.finalize().as_bytes())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct BrokerProcessSnapshot {
    pid: u32,
    process_start_time_ticks: u64,
    effective_uid: u32,
    executable_path: String,
    executable_digest: [u8; 32],
    network_namespace: NamespaceIdentity,
    snapshot_digest: [u8; 32],
}

impl BrokerProcessSnapshot {
    pub fn pid(&self) -> u32 {
        self.pid
    }

    pub fn process_start_time_ticks(&self) -> u64 {
        self.process_start_time_ticks
    }

    pub fn effective_uid(&self) -> u32 {
        self.effective_uid
    }

    pub fn executable_path(&self) -> &str {
        &self.executable_path
    }

    pub fn executable_digest(&self) -> [u8; 32] {
        self.executable_digest
    }

    pub fn network_namespace(&self) -> NamespaceIdentity {
        self.network_namespace
    }

    pub fn snapshot_digest(&self) -> [u8; 32] {
        self.snapshot_digest
    }

    pub fn snapshot_profile(&self) -> &str {
        BROKER_SNAPSHOT_PROFILE
    }
}

/// Positive historical proof that the pinned broker and pinned Admin listener
/// stayed inside one isolated loopback-only namespace over the guarded interval.
#[derive(Clone, Debug, Serialize)]
pub struct QualifiedExclusiveAdminIsolation {
    broker_binding_digest: [u8; 32],
    broker_snapshot: BrokerProcessSnapshot,
    conductor_fence: QualifiedConductorProcessFence,
    network_namespace: NamespaceIdentity,
    member_pids: Vec<u32>,
    interfaces: Vec<String>,
    host_network_namespace: NamespaceIdentity,
    isolation_digest: [u8; 32],
    started_at_ms: u64,
    ended_at_ms: u64,
}

impl QualifiedExclusiveAdminIsolation {
    pub fn broker_binding_digest(&self) -> [u8; 32] {
        self.broker_binding_digest
    }

    pub fn broker_snapshot(&self) -> &BrokerProcessSnapshot {
        &self.broker_snapshot
    }

    pub fn conductor_fence(&self) -> &QualifiedConductorProcessFence {
        &self.conductor_fence
    }

    pub fn network_namespace(&self) -> NamespaceIdentity {
        self.network_namespace
    }

    pub fn member_pids(&self) -> &[u32] {
        &self.member_pids
    }

    pub fn interfaces(&self) -> &[String] {
        &self.interfaces
    }

    pub fn isolation_digest(&self) -> [u8; 32] {
        self.isolation_digest
    }

    pub fn isolation_profile(&self) -> &str {
        ISOLATION_PROFILE
    }

    pub fn started_at_ms(&self) -> u64 {
        self.started_at_ms
    }

    pub fn ended_at_ms(&self) -> u64 {
        self.ended_at_ms
    }

    pub const fn broker_and_conductor_namespace_bound_here(&self) -> bool {
        true
    }

    pub const fn host_loopback_bypass_excluded_here(&self) -> bool {
        true
    }

    pub const fn coordinator_update_excluded_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Caller-controlled isolation interval. The existing conductor fence remains
/// open while this guard lives, so later work can be bracketed by both process
/// identity and network-namespace isolation.
pub struct ExclusiveAdminIsolationGuard<'a> {
    conductor_guard: ConductorProcessFenceGuard<'a>,
    binding: BrokerProcessBinding,
    binding_digest: [u8; 32],
    broker_pre: BrokerProcessSnapshot,
    network_namespace: NamespaceIdentity,
    host_network_namespace: NamespaceIdentity,
    pre_member_pids: Vec<u32>,
    pre_interfaces: Vec<String>,
    started_at_ms: u64,
}

pub fn begin_exclusive_admin_isolation<'a>(
    conductor_store: &'a TrustedConductorProcessStore,
    broker_binding: &BrokerProcessBinding,
) -> Result<ExclusiveAdminIsolationGuard<'a>, AdminIsolationError> {
    broker_binding.validate()?;
    let binding_digest = broker_binding.binding_digest()?;

    // Start #381 first so listener/process identity stays fenced throughout every
    // namespace observation and any caller-controlled work before `finish()`.
    let conductor_guard = conductor_store
        .begin_fence()
        .map_err(AdminIsolationError::ConductorProcess)?;
    if conductor_guard.admin_endpoint() != broker_binding.admin_endpoint {
        return Err(AdminIsolationError::AdminEndpointMismatch);
    }

    let broker_pre = observe_broker_process(broker_binding)?;
    let host_network_namespace = namespace_identity("/proc/1/ns/net")?;
    if broker_pre.network_namespace == host_network_namespace {
        return Err(AdminIsolationError::HostNetworkNamespace);
    }

    let pre_interfaces = network_interfaces()?;
    require_loopback_only(&pre_interfaces)?;
    let pre_member_pids = namespace_members(broker_pre.network_namespace)?;
    require_two_member_namespace(&pre_member_pids, broker_pre.pid)?;

    let started_at_ms = now_ms()?;
    if started_at_ms < conductor_guard.started_at_ms() {
        return Err(AdminIsolationError::InvalidInterval);
    }

    Ok(ExclusiveAdminIsolationGuard {
        conductor_guard,
        binding: broker_binding.clone(),
        binding_digest,
        broker_pre,
        network_namespace: broker_pre.network_namespace,
        host_network_namespace,
        pre_member_pids,
        pre_interfaces,
        started_at_ms,
    })
}

impl ExclusiveAdminIsolationGuard<'_> {
    pub fn admin_endpoint(&self) -> SocketAddr {
        self.binding.admin_endpoint
    }

    pub fn broker_pid(&self) -> u32 {
        self.broker_pre.pid
    }

    pub fn network_namespace(&self) -> NamespaceIdentity {
        self.network_namespace
    }

    /// Close the caller-controlled interval and recheck every isolation fact.
    pub fn finish(self) -> Result<QualifiedExclusiveAdminIsolation, AdminIsolationError> {
        let broker_mid = observe_broker_process(&self.binding)?;
        require_same_broker(&self.broker_pre, &broker_mid)?;
        let mid_interfaces = network_interfaces()?;
        require_loopback_only(&mid_interfaces)?;
        let mid_members = namespace_members(self.network_namespace)?;
        if mid_interfaces != self.pre_interfaces || mid_members != self.pre_member_pids {
            return Err(AdminIsolationError::NamespaceChanged);
        }

        let conductor_fence = self
            .conductor_guard
            .finish()
            .map_err(AdminIsolationError::ConductorProcess)?;
        if conductor_fence.admin_endpoint() != self.binding.admin_endpoint {
            return Err(AdminIsolationError::AdminEndpointMismatch);
        }
        if conductor_fence.binding_digest() != self.binding.conductor_binding_digest {
            return Err(AdminIsolationError::ConductorBindingMismatch);
        }

        // Re-read after the conductor fence closes as well. This prevents a
        // successful #381 close from being followed by a silently different
        // broker/namespace snapshot before this theorem is constructed.
        let broker_post = observe_broker_process(&self.binding)?;
        require_same_broker(&self.broker_pre, &broker_post)?;
        let post_interfaces = network_interfaces()?;
        require_loopback_only(&post_interfaces)?;
        let post_members = namespace_members(self.network_namespace)?;
        if post_interfaces != self.pre_interfaces || post_members != self.pre_member_pids {
            return Err(AdminIsolationError::NamespaceChanged);
        }

        let conductor_pid = conductor_fence.process_snapshot().pid;
        let mut expected_members = vec![self.broker_pre.pid, conductor_pid];
        expected_members.sort_unstable();
        expected_members.dedup();
        if expected_members.len() != 2 || post_members != expected_members {
            return Err(AdminIsolationError::UnexpectedNamespaceMemberSet);
        }

        let ended_at_ms = now_ms()?;
        if ended_at_ms < self.started_at_ms || conductor_fence.ended_at_ms() > ended_at_ms {
            return Err(AdminIsolationError::InvalidInterval);
        }

        let isolation_digest = isolation_digest(
            self.binding_digest,
            &broker_post,
            &conductor_fence,
            self.network_namespace,
            self.host_network_namespace,
            &post_members,
            &post_interfaces,
            self.started_at_ms,
            ended_at_ms,
        );

        Ok(QualifiedExclusiveAdminIsolation {
            broker_binding_digest: self.binding_digest,
            broker_snapshot: broker_post,
            conductor_fence,
            network_namespace: self.network_namespace,
            member_pids: post_members,
            interfaces: post_interfaces,
            host_network_namespace: self.host_network_namespace,
            isolation_digest,
            started_at_ms: self.started_at_ms,
            ended_at_ms,
        })
    }
}

fn observe_broker_process(
    binding: &BrokerProcessBinding,
) -> Result<BrokerProcessSnapshot, AdminIsolationError> {
    let pid = std::process::id();
    if pid == 0 {
        return Err(AdminIsolationError::InvalidBrokerProcess);
    }
    let effective_uid = unsafe { libc::geteuid() };
    if effective_uid != binding.expected_broker_uid {
        return Err(AdminIsolationError::BrokerUidMismatch {
            expected: binding.expected_broker_uid,
            observed: effective_uid,
        });
    }

    let executable_path = fs::read_link("/proc/self/exe")
        .map_err(AdminIsolationError::Io)?
        .to_string_lossy()
        .into_owned();
    if executable_path != binding.broker_executable_path {
        return Err(AdminIsolationError::BrokerExecutablePathMismatch);
    }
    let executable_digest = digest_file("/proc/self/exe")?;
    if executable_digest != binding.broker_executable_digest {
        return Err(AdminIsolationError::BrokerExecutableDigestMismatch);
    }

    let process_start_time_ticks = process_start_time_ticks("/proc/self/stat")?;
    let network_namespace = namespace_identity("/proc/self/ns/net")?;
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_SNAPSHOT);
    frame(&mut h, BROKER_SNAPSHOT_PROFILE.as_bytes());
    frame(&mut h, &pid.to_le_bytes());
    frame(&mut h, &process_start_time_ticks.to_le_bytes());
    frame(&mut h, &effective_uid.to_le_bytes());
    frame(&mut h, executable_path.as_bytes());
    frame(&mut h, &executable_digest);
    frame(&mut h, &network_namespace.device.to_le_bytes());
    frame(&mut h, &network_namespace.inode.to_le_bytes());
    let snapshot_digest = *h.finalize().as_bytes();

    Ok(BrokerProcessSnapshot {
        pid,
        process_start_time_ticks,
        effective_uid,
        executable_path,
        executable_digest,
        network_namespace,
        snapshot_digest,
    })
}

fn require_same_broker(
    expected: &BrokerProcessSnapshot,
    observed: &BrokerProcessSnapshot,
) -> Result<(), AdminIsolationError> {
    if expected.pid != observed.pid
        || expected.process_start_time_ticks != observed.process_start_time_ticks
        || expected.effective_uid != observed.effective_uid
        || expected.executable_path != observed.executable_path
        || expected.executable_digest != observed.executable_digest
        || expected.network_namespace != observed.network_namespace
    {
        Err(AdminIsolationError::BrokerProcessChanged)
    } else {
        Ok(())
    }
}

fn namespace_identity(path: &str) -> Result<NamespaceIdentity, AdminIsolationError> {
    let metadata = fs::metadata(path).map_err(AdminIsolationError::Io)?;
    let identity = NamespaceIdentity {
        device: metadata.dev(),
        inode: metadata.ino(),
    };
    identity.validate()?;
    Ok(identity)
}

fn namespace_members(namespace: NamespaceIdentity) -> Result<Vec<u32>, AdminIsolationError> {
    namespace.validate()?;
    let mut members = Vec::new();
    for entry in fs::read_dir("/proc").map_err(AdminIsolationError::Io)? {
        let entry = entry.map_err(AdminIsolationError::Io)?;
        let name = entry.file_name();
        let Some(name) = name.to_str() else {
            continue;
        };
        let Ok(pid) = name.parse::<u32>() else {
            continue;
        };
        let path = format!("/proc/{pid}/ns/net");
        match fs::metadata(&path) {
            Ok(metadata) => {
                if metadata.dev() == namespace.device && metadata.ino() == namespace.inode {
                    members.push(pid);
                    if members.len() > MAX_NAMESPACE_MEMBERS {
                        return Err(AdminIsolationError::TooManyNamespaceMembers);
                    }
                }
            }
            Err(error) if error.kind() == io::ErrorKind::NotFound => {}
            Err(error) => return Err(AdminIsolationError::Io(error)),
        }
    }
    members.sort_unstable();
    members.dedup();
    Ok(members)
}

fn require_two_member_namespace(members: &[u32], broker_pid: u32) -> Result<(), AdminIsolationError> {
    if members.len() != 2 || !members.contains(&broker_pid) {
        Err(AdminIsolationError::UnexpectedNamespaceMemberSet)
    } else {
        Ok(())
    }
}

fn network_interfaces() -> Result<Vec<String>, AdminIsolationError> {
    let text = fs::read_to_string("/proc/self/net/dev").map_err(AdminIsolationError::Io)?;
    parse_interface_names(&text)
}

fn parse_interface_names(text: &str) -> Result<Vec<String>, AdminIsolationError> {
    let mut interfaces = Vec::new();
    for line in text.lines() {
        let Some((name, _)) = line.split_once(':') else {
            continue;
        };
        let name = name.trim();
        if name.is_empty() {
            return Err(AdminIsolationError::InvalidInterfaceTable);
        }
        interfaces.push(name.to_string());
    }
    interfaces.sort();
    interfaces.dedup();
    if interfaces.is_empty() {
        return Err(AdminIsolationError::InvalidInterfaceTable);
    }
    Ok(interfaces)
}

fn require_loopback_only(interfaces: &[String]) -> Result<(), AdminIsolationError> {
    if interfaces.len() == 1 && interfaces[0] == "lo" {
        Ok(())
    } else {
        Err(AdminIsolationError::NonLoopbackInterfacePresent)
    }
}

fn process_start_time_ticks(path: &str) -> Result<u64, AdminIsolationError> {
    let stat = fs::read_to_string(path).map_err(AdminIsolationError::Io)?;
    parse_process_start_time_ticks(&stat)
}

fn parse_process_start_time_ticks(stat: &str) -> Result<u64, AdminIsolationError> {
    let close = stat
        .rfind(')')
        .ok_or(AdminIsolationError::InvalidProcStat)?;
    let tail = stat
        .get(close + 1..)
        .ok_or(AdminIsolationError::InvalidProcStat)?;
    // After the comm field, token 0 is field 3 (`state`); starttime is field 22.
    tail.split_whitespace()
        .nth(19)
        .ok_or(AdminIsolationError::InvalidProcStat)?
        .parse::<u64>()
        .map_err(|_| AdminIsolationError::InvalidProcStat)
        .and_then(|value| {
            if value == 0 {
                Err(AdminIsolationError::InvalidProcStat)
            } else {
                Ok(value)
            }
        })
}

fn digest_file(path: &str) -> Result<[u8; 32], AdminIsolationError> {
    let mut file = File::open(path).map_err(AdminIsolationError::Io)?;
    let metadata = file.metadata().map_err(AdminIsolationError::Io)?;
    if metadata.len() == 0 || metadata.len() > MAX_EXECUTABLE_BYTES {
        return Err(AdminIsolationError::InvalidBrokerExecutableLength);
    }
    let mut hasher = blake3::Hasher::new();
    let mut buffer = [0_u8; 64 * 1024];
    loop {
        let read = file.read(&mut buffer).map_err(AdminIsolationError::Io)?;
        if read == 0 {
            break;
        }
        hasher.update(&buffer[..read]);
    }
    Ok(*hasher.finalize().as_bytes())
}

#[allow(clippy::too_many_arguments)]
fn isolation_digest(
    broker_binding_digest: [u8; 32],
    broker: &BrokerProcessSnapshot,
    conductor: &QualifiedConductorProcessFence,
    namespace: NamespaceIdentity,
    host_namespace: NamespaceIdentity,
    member_pids: &[u32],
    interfaces: &[String],
    started_at_ms: u64,
    ended_at_ms: u64,
) -> [u8; 32] {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_ISOLATION);
    frame(&mut h, PROTOCOL_VERSION.as_bytes());
    frame(&mut h, ISOLATION_PROFILE.as_bytes());
    frame(&mut h, &broker_binding_digest);
    frame(&mut h, &broker.snapshot_digest);
    frame(&mut h, BROKER_SNAPSHOT_PROFILE.as_bytes());
    frame(&mut h, &conductor.qualification_digest());
    frame(&mut h, conductor.qualification_profile().as_bytes());
    frame(&mut h, &namespace.device.to_le_bytes());
    frame(&mut h, &namespace.inode.to_le_bytes());
    frame(&mut h, &host_namespace.device.to_le_bytes());
    frame(&mut h, &host_namespace.inode.to_le_bytes());
    frame(&mut h, &(member_pids.len() as u64).to_le_bytes());
    for pid in member_pids {
        frame(&mut h, &pid.to_le_bytes());
    }
    frame(&mut h, &(interfaces.len() as u64).to_le_bytes());
    for interface in interfaces {
        frame(&mut h, interface.as_bytes());
    }
    frame(&mut h, &started_at_ms.to_le_bytes());
    frame(&mut h, &ended_at_ms.to_le_bytes());
    *h.finalize().as_bytes()
}

fn now_ms() -> Result<u64, AdminIsolationError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| AdminIsolationError::ClockBeforeUnixEpoch)?;
    let value = u64::try_from(duration.as_millis()).map_err(|_| AdminIsolationError::ClockOverflow)?;
    if value == 0 {
        Err(AdminIsolationError::ClockBeforeUnixEpoch)
    } else {
        Ok(value)
    }
}

fn validate_digest(value: &[u8; 32], field: &'static str) -> Result<(), AdminIsolationError> {
    if value.iter().all(|byte| *byte == 0) {
        Err(AdminIsolationError::InvalidDigest(field))
    } else {
        Ok(())
    }
}

fn validate_text(value: &str, field: &'static str) -> Result<(), AdminIsolationError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES {
        Err(AdminIsolationError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Debug)]
pub enum AdminIsolationError {
    WrongProtocol,
    InvalidText(&'static str),
    InvalidDigest(&'static str),
    InvalidAdminEndpoint,
    BrokerExecutablePathNotAbsolute,
    WrongBrokerExecutableProfile,
    InvalidBrokerProcess,
    BrokerUidMismatch { expected: u32, observed: u32 },
    BrokerExecutablePathMismatch,
    BrokerExecutableDigestMismatch,
    InvalidBrokerExecutableLength,
    InvalidProcStat,
    InvalidNamespaceIdentity,
    HostNetworkNamespace,
    TooManyNamespaceMembers,
    UnexpectedNamespaceMemberSet,
    NonLoopbackInterfacePresent,
    InvalidInterfaceTable,
    BrokerProcessChanged,
    NamespaceChanged,
    AdminEndpointMismatch,
    ConductorBindingMismatch,
    InvalidInterval,
    ClockBeforeUnixEpoch,
    ClockOverflow,
    Io(io::Error),
    ConductorProcess(ConductorProcessError),
}

impl fmt::Display for AdminIsolationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocol => write!(f, "wrong coordinator Admin-isolation protocol"),
            Self::InvalidText(field) => write!(f, "invalid {field}"),
            Self::InvalidDigest(field) => write!(f, "invalid {field}"),
            Self::InvalidAdminEndpoint => write!(f, "Admin endpoint must be non-zero loopback"),
            Self::BrokerExecutablePathNotAbsolute => write!(f, "broker executable path is not absolute"),
            Self::WrongBrokerExecutableProfile => write!(f, "wrong broker executable profile"),
            Self::InvalidBrokerProcess => write!(f, "invalid broker process"),
            Self::BrokerUidMismatch { expected, observed } => {
                write!(f, "broker UID mismatch: expected {expected}, observed {observed}")
            }
            Self::BrokerExecutablePathMismatch => write!(f, "broker executable path mismatch"),
            Self::BrokerExecutableDigestMismatch => write!(f, "broker executable digest mismatch"),
            Self::InvalidBrokerExecutableLength => write!(f, "invalid broker executable length"),
            Self::InvalidProcStat => write!(f, "invalid Linux proc stat data"),
            Self::InvalidNamespaceIdentity => write!(f, "invalid network namespace identity"),
            Self::HostNetworkNamespace => write!(f, "broker still occupies the host network namespace"),
            Self::TooManyNamespaceMembers => write!(f, "network namespace member count exceeds v0.1 bound"),
            Self::UnexpectedNamespaceMemberSet => write!(f, "network namespace is not exactly broker + conductor"),
            Self::NonLoopbackInterfacePresent => write!(f, "isolated Admin namespace exposes a non-loopback interface"),
            Self::InvalidInterfaceTable => write!(f, "invalid network interface table"),
            Self::BrokerProcessChanged => write!(f, "broker process identity changed during isolation interval"),
            Self::NamespaceChanged => write!(f, "network namespace membership/interfaces changed during interval"),
            Self::AdminEndpointMismatch => write!(f, "broker and conductor do not name the exact same Admin endpoint"),
            Self::ConductorBindingMismatch => write!(f, "conductor fence does not match the pinned broker control-plane binding"),
            Self::InvalidInterval => write!(f, "invalid Admin-isolation interval"),
            Self::ClockBeforeUnixEpoch => write!(f, "system clock is before Unix epoch"),
            Self::ClockOverflow => write!(f, "system clock does not fit u64 milliseconds"),
            Self::Io(error) => write!(f, "Linux isolation observation failed: {error}"),
            Self::ConductorProcess(error) => write!(f, "conductor process fence failed: {error}"),
        }
    }
}

impl std::error::Error for AdminIsolationError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn binding() -> BrokerProcessBinding {
        BrokerProcessBinding {
            protocol_version: PROTOCOL_VERSION.into(),
            binding_id: "broker:primary".into(),
            admin_endpoint: "127.0.0.1:4444".parse().unwrap(),
            expected_broker_uid: 1000,
            broker_executable_path: "/nix/store/example/bin/mycelix-admin-broker".into(),
            broker_executable_digest: d(1),
            broker_executable_profile: EXECUTABLE_PROFILE.into(),
            conductor_binding_digest: d(2),
            binding_ref: "provisioning:broker:1".into(),
        }
    }

    #[test]
    fn binding_identity_is_stable_and_endpoint_is_loopback() {
        let a = binding();
        let b = a.clone();
        assert_eq!(a.binding_digest().unwrap(), b.binding_digest().unwrap());
    }

    #[test]
    fn non_loopback_admin_endpoint_denies() {
        let mut value = binding();
        value.admin_endpoint = "192.0.2.5:4444".parse().unwrap();
        assert!(matches!(
            value.validate().unwrap_err(),
            AdminIsolationError::InvalidAdminEndpoint
        ));
    }

    #[test]
    fn interface_parser_requires_real_interface_rows() {
        let fixture = "Inter-| Receive | Transmit\n face |bytes|bytes\n    lo: 1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0\n";
        assert_eq!(parse_interface_names(fixture).unwrap(), vec!["lo"]);
        require_loopback_only(&parse_interface_names(fixture).unwrap()).unwrap();
    }

    #[test]
    fn extra_interface_denies_loopback_only_theorem() {
        let fixture = "lo: 1\neth0: 2\n";
        let interfaces = parse_interface_names(fixture).unwrap();
        assert!(matches!(
            require_loopback_only(&interfaces).unwrap_err(),
            AdminIsolationError::NonLoopbackInterfacePresent
        ));
    }

    #[test]
    fn proc_stat_parser_handles_spaces_in_comm() {
        let mut fields = vec!["S".to_string()];
        fields.extend((4_u64..=21).map(|value| value.to_string()));
        fields.push("424242".into());
        fields.push("23".into());
        let stat = format!("99 (broker with spaces) {}", fields.join(" "));
        assert_eq!(parse_process_start_time_ticks(&stat).unwrap(), 424242);
    }

    #[test]
    fn binding_digest_changes_with_conductor_binding() {
        let a = binding();
        let mut b = a.clone();
        b.conductor_binding_digest = d(9);
        assert_ne!(a.binding_digest().unwrap(), b.binding_digest().unwrap());
    }
}
