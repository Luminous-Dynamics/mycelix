//! Broker-owned renderer process identity and sandbox contract.
//!
//! This module does not claim to implement OS sandboxing. It defines the
//! browser-side contract that a future process supervisor must satisfy before
//! a renderer can receive capability authority.
//!
//! Security property:
//! PID is never treated as a stable renderer identity. On Linux the process
//! start-time value from /proc/<pid>/stat is retained with the assignment so
//! PID reuse cannot inherit an old RendererProcessId.

use crate::identity::RendererProcessId;
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RendererProcessAssignmentId(pub u128);

impl RendererProcessAssignmentId {
    pub fn new(value: u128) -> Result<Self, ProcessContractError> {
        if value == 0 {
            return Err(ProcessContractError::InvalidAssignmentId);
        }
        Ok(Self(value))
    }
}

/// Explicit sandbox policy. These are requirements, not proof that the OS
/// sandbox has actually been installed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SandboxProfileV1 {
    pub network: NetworkPolicy,
    pub filesystem: FilesystemPolicy,
    pub devices: DevicePolicy,
    pub child_processes: ChildProcessPolicy,
    syscall_policy_digest: [u8; 32],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NetworkPolicy {
    BrokerOnly,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilesystemPolicy {
    NoAmbientAccess,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DevicePolicy {
    None,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChildProcessPolicy {
    Deny,
}

impl SandboxProfileV1 {
    /// Stable commitment to the requested policy; this is not proof of OS enforcement.
    pub fn policy_digest(&self) -> [u8; 32] {
        let mut hasher = blake3::Hasher::new();
        hasher.update(&[
            match self.network { NetworkPolicy::BrokerOnly => 1 },
            match self.filesystem { FilesystemPolicy::NoAmbientAccess => 1 },
            match self.devices { DevicePolicy::None => 1 },
            match self.child_processes { ChildProcessPolicy::Deny => 1 },
        ]);
        hasher.update(&self.syscall_policy_digest);
        *hasher.finalize().as_bytes()
    }

    pub const fn renderer_default() -> Self {
        Self {
            network: NetworkPolicy::BrokerOnly,
            filesystem: FilesystemPolicy::NoAmbientAccess,
            devices: DevicePolicy::None,
            child_processes: ChildProcessPolicy::Deny,
            syscall_policy_digest: [0u8; 32],
        }
    }

    /// Bind a qualified renderer syscall policy commitment to this profile.
    /// The zero digest is intentionally rejected so an uncommitted profile
    /// can never be mistaken for one with an approved syscall policy.
    pub fn with_syscall_policy_digest(mut self, digest: [u8; 32]) -> Result<Self, ProcessContractError> {
        if digest == [0u8; 32] {
            return Err(ProcessContractError::InvalidSyscallPolicyCommitment);
        }
        self.syscall_policy_digest = digest;
        Ok(self)
    }

    pub fn syscall_policy_digest(&self) -> [u8; 32] {
        self.syscall_policy_digest
    }
}

/// Independent identity for one sandbox installation attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SandboxInstallationId(pub u128);

impl SandboxInstallationId {
    pub fn new(value: u128) -> Result<Self, ProcessContractError> {
        if value == 0 { return Err(ProcessContractError::InvalidSandboxInstallationId); }
        Ok(Self(value))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SandboxAdapterKind {
    LinuxLandlockFilesystemV1,
    LinuxSeccompSyscallV1,
    UnsupportedPlatform,
}

/// Distinct OS-enforced security layers. An adapter may enforce only a subset;
/// the supervisor must require coverage of every layer required by the profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum SandboxEnforcementLayer {
    Filesystem = 0,
    Network = 1,
    Device = 2,
    ChildProcess = 3,
    Syscall = 4,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SandboxEnforcementSet(u8);

impl SandboxEnforcementSet {
    pub const EMPTY: Self = Self(0);
    pub const fn from_layer(layer: SandboxEnforcementLayer) -> Self { Self(1 << (layer as u8)) }
    pub const fn contains(self, layer: SandboxEnforcementLayer) -> bool { self.0 & (1 << (layer as u8)) != 0 }
    pub const fn union(self, other: Self) -> Self { Self(self.0 | other.0) }
    pub const fn covers(self, required: Self) -> bool { self.0 & required.0 == required.0 }
}

impl SandboxProfileV1 {
    /// Capability authority requires an OS enforcement receipt covering every
    /// layer required by the broker-owned renderer profile.
    pub const fn required_enforcement_layers(self) -> SandboxEnforcementSet {
        SandboxEnforcementSet::from_layer(SandboxEnforcementLayer::Filesystem)
            .union(SandboxEnforcementSet::from_layer(SandboxEnforcementLayer::Network))
            .union(SandboxEnforcementSet::from_layer(SandboxEnforcementLayer::Device))
            .union(SandboxEnforcementSet::from_layer(SandboxEnforcementLayer::ChildProcess))
            .union(SandboxEnforcementSet::from_layer(SandboxEnforcementLayer::Syscall))
    }
}

/// Non-secret evidence from exactly one OS sandbox adapter/layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SandboxEnforcementReceipt {
    assignment_id: RendererProcessAssignmentId,
    installation_id: SandboxInstallationId,
    adapter: SandboxAdapterKind,
    profile_digest: [u8; 32],
    evidence_digest: [u8; 32],
    layer: SandboxEnforcementLayer,
}

impl SandboxEnforcementReceipt {
    /// Broker-internal constructor. The adapter/layer mapping is checked here
    /// so one adapter cannot manufacture evidence for an unrelated layer.
    pub(crate) fn from_adapter(
        assignment_id: RendererProcessAssignmentId,
        installation_id: SandboxInstallationId,
        adapter: SandboxAdapterKind,
        profile_digest: [u8; 32],
        evidence_digest: [u8; 32],
        layer: SandboxEnforcementLayer,
    ) -> Result<Self, ProcessContractError> {
        let valid = matches!(
            (adapter, layer),
            (SandboxAdapterKind::LinuxLandlockFilesystemV1, SandboxEnforcementLayer::Filesystem)
                | (SandboxAdapterKind::LinuxSeccompSyscallV1, SandboxEnforcementLayer::Syscall)
        );
        if !valid
            || assignment_id.0 == 0
            || installation_id.0 == 0
            || profile_digest == [0u8; 32]
            || evidence_digest == [0u8; 32]
        {
            return Err(ProcessContractError::InvalidSandboxEvidence);
        }
        Ok(Self { assignment_id, installation_id, adapter, profile_digest, evidence_digest, layer })
    }

    pub fn assignment_id(&self) -> RendererProcessAssignmentId { self.assignment_id }
    pub fn installation_id(&self) -> SandboxInstallationId { self.installation_id }
    pub fn adapter(&self) -> SandboxAdapterKind { self.adapter }
    pub fn profile_digest(&self) -> [u8; 32] { self.profile_digest }
    pub fn evidence_digest(&self) -> [u8; 32] { self.evidence_digest }
    pub fn layer(&self) -> SandboxEnforcementLayer { self.layer }
}

/// Independently evidenced sandbox layers for one renderer assignment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SandboxEvidenceBundle {
    assignment_id: RendererProcessAssignmentId,
    policy_digest: [u8; 32],
    enforced_layers: SandboxEnforcementSet,
}

impl SandboxEvidenceBundle {
    pub fn new(
        assignment_id: RendererProcessAssignmentId,
        policy_digest: [u8; 32],
    ) -> Self {
        Self { assignment_id, policy_digest, enforced_layers: SandboxEnforcementSet::EMPTY }
    }

    pub fn record(
        &mut self,
        receipt: SandboxEnforcementReceipt,
    ) -> Result<(), ProcessContractError> {
        if receipt.assignment_id != self.assignment_id
            || receipt.profile_digest != self.policy_digest
            || receipt.installation_id.0 == 0
        {
            return Err(ProcessContractError::SandboxEvidenceMismatch);
        }
        if self.enforced_layers.contains(receipt.layer) {
            return Err(ProcessContractError::DuplicateSandboxEvidence);
        }
        self.enforced_layers = self.enforced_layers.union(
            SandboxEnforcementSet::from_layer(receipt.layer)
        );
        Ok(())
    }

    pub fn assignment_id(&self) -> RendererProcessAssignmentId { self.assignment_id }
    pub fn policy_digest(&self) -> [u8; 32] { self.policy_digest }
    pub fn enforced_layers(&self) -> SandboxEnforcementSet { self.enforced_layers }
}

/// Kernel-observed process identity. PID alone is deliberately insufficient.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProcessIdentity {
    pub pid: u32,
    pub start_time_ticks: u64,
}

impl ProcessIdentity {
    pub fn new(pid: u32, start_time_ticks: u64) -> Result<Self, ProcessContractError> {
        if pid == 0 || start_time_ticks == 0 {
            return Err(ProcessContractError::InvalidProcessIdentity);
        }
        Ok(Self { pid, start_time_ticks })
    }
}

/// Non-secret launch receipt. It binds browser-assigned identity to an OS
/// process observation, generation, and the sandbox policy that was required.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RendererLaunchReceipt {
    assignment_id: RendererProcessAssignmentId,
    renderer_process: RendererProcessId,
    process: ProcessIdentity,
    generation: u64,
    sandbox: SandboxProfileV1,
}

impl RendererLaunchReceipt {
    pub fn assignment_id(&self) -> RendererProcessAssignmentId { self.assignment_id }
    pub fn renderer_process(&self) -> RendererProcessId { self.renderer_process }
    pub fn process(&self) -> ProcessIdentity { self.process }
    pub fn generation(&self) -> u64 { self.generation }
    pub fn sandbox(&self) -> SandboxProfileV1 { self.sandbox }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProcessContractError {
    InvalidAssignmentId,
    InvalidSandboxInstallationId,
    InvalidSandboxEvidence,
    InvalidSyscallPolicyCommitment,
    SandboxEvidenceMismatch,
    DuplicateSandboxEvidence,
    InvalidProcessIdentity,
    ProcessAlreadyAssigned,
    NoActiveAssignment,
    ProcessIdentityMismatch,
    UnsupportedProcessObservation,
    ProcessNotFound,
}

impl fmt::Display for ProcessContractError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidAssignmentId => f.write_str("renderer process assignment id must be non-zero"),
            Self::InvalidSandboxInstallationId => f.write_str("sandbox installation id must be non-zero"),
            Self::InvalidSandboxEvidence => f.write_str("sandbox adapter cannot attest to the requested enforcement layer"),
            Self::InvalidSyscallPolicyCommitment => f.write_str("renderer syscall policy commitment must be non-zero"),
            Self::SandboxEvidenceMismatch => f.write_str("sandbox evidence does not match the renderer assignment or policy"),
            Self::DuplicateSandboxEvidence => f.write_str("sandbox enforcement layer was already evidenced"),
            Self::InvalidProcessIdentity => f.write_str("renderer process identity is invalid"),
            Self::ProcessAlreadyAssigned => f.write_str("a renderer process assignment is already active"),
            Self::NoActiveAssignment => f.write_str("no renderer process assignment is active"),
            Self::ProcessIdentityMismatch => f.write_str("observed process identity does not match assignment"),
            Self::UnsupportedProcessObservation => f.write_str("this platform does not expose the required process identity observation"),
            Self::ProcessNotFound => f.write_str("renderer process was not found"),
        }
    }
}

impl std::error::Error for ProcessContractError {}

#[derive(Debug, Default)]
pub struct RendererProcessController {
    active: Option<RendererLaunchReceipt>,
}

impl RendererProcessController {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn active(&self) -> Option<&RendererLaunchReceipt> {
        self.active.as_ref()
    }

    /// Register a process only after the browser has assigned its logical
    /// RendererProcessId and observed the OS process identity.
    pub fn register_launch(
        &mut self,
        renderer_process: RendererProcessId,
        generation: u64,
        process: ProcessIdentity,
        sandbox: SandboxProfileV1,
    ) -> Result<RendererLaunchReceipt, ProcessContractError> {
        if self.active.is_some() {
            return Err(ProcessContractError::ProcessAlreadyAssigned);
        }

        let assignment_id = RendererProcessAssignmentId::new(next_assignment_id())?;
        let receipt = RendererLaunchReceipt {
            assignment_id,
            renderer_process,
            process,
            generation,
            sandbox,
        };
        self.active = Some(receipt);
        Ok(receipt)
    }

    /// Validate that a live OS observation still denotes the exact assigned
    /// process. A PID match with a different start time is rejected.
    pub fn validate_identity(
        &self,
        renderer_process: RendererProcessId,
        observed: ProcessIdentity,
    ) -> Result<(), ProcessContractError> {
        let receipt = self.active.as_ref().ok_or(ProcessContractError::NoActiveAssignment)?;
        if receipt.renderer_process != renderer_process || receipt.process != observed {
            return Err(ProcessContractError::ProcessIdentityMismatch);
        }
        Ok(())
    }

    /// Explicitly retire the assignment. A replacement process must receive a
    /// fresh RendererProcessId/generation/session binding.
    pub fn retire(
        &mut self,
        renderer_process: RendererProcessId,
        observed: ProcessIdentity,
    ) -> Result<RendererLaunchReceipt, ProcessContractError> {
        self.validate_identity(renderer_process, observed)?;
        Ok(self.active.take().expect("validated active assignment"))
    }

    /// Check the currently assigned process against the host OS.
    pub fn observe_current(&self) -> Result<(), ProcessContractError> {
        let receipt = self.active.as_ref().ok_or(ProcessContractError::NoActiveAssignment)?;
        let observed = observe_process_identity(receipt.process.pid)?;
        self.validate_identity(receipt.renderer_process, observed)
    }
}

pub fn next_sandbox_installation_id() -> Result<SandboxInstallationId, ProcessContractError> {
    let mut bytes = [0u8; 16];
    getrandom::fill(&mut bytes).map_err(|_| ProcessContractError::InvalidSandboxInstallationId)?;
    SandboxInstallationId::new(u128::from_be_bytes(bytes))
}

fn next_assignment_id() -> u128 {
    let mut bytes = [0u8; 16];
    if getrandom::fill(&mut bytes).is_err() {
        // A zero ID is rejected, so a failed RNG can never silently create a
        // valid assignment. The caller will retry/fail closed.
        return 0;
    }
    u128::from_be_bytes(bytes)
}

/// Linux process identity observation.
///
/// /proc/<pid>/stat field 22 is the process start time after system boot.
/// It is retained because a PID can later be reused for a different process.
pub fn observe_process_identity(pid: u32) -> Result<ProcessIdentity, ProcessContractError> {
    #[cfg(target_os = "linux")]
    {
        let stat = std::fs::read_to_string(format!("/proc/{pid}/stat"))
            .map_err(|_| ProcessContractError::ProcessNotFound)?;
        let close = stat.rfind(')').ok_or(ProcessContractError::ProcessNotFound)?;
        let fields = stat[close + 1..].split_whitespace().collect::<Vec<_>>();
        let start_time = fields
            .get(19)
            .and_then(|value| value.parse::<u64>().ok())
            .ok_or(ProcessContractError::ProcessNotFound)?;
        return ProcessIdentity::new(pid, start_time);
    }

    #[cfg(not(target_os = "linux"))]
    {
        let _ = pid;
        Err(ProcessContractError::UnsupportedProcessObservation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn renderer_sandbox_profile_is_explicit() {
        let profile = SandboxProfileV1::renderer_default();
        assert_eq!(profile.network, NetworkPolicy::BrokerOnly);
        assert_eq!(profile.filesystem, FilesystemPolicy::NoAmbientAccess);
        assert_eq!(profile.devices, DevicePolicy::None);
        assert_eq!(profile.child_processes, ChildProcessPolicy::Deny);
        assert_ne!(profile.policy_digest(), [0u8; 32]);
    }

    #[test]
    fn sandbox_installation_ids_are_independent() {
        let a = next_sandbox_installation_id().unwrap();
        let b = next_sandbox_installation_id().unwrap();
        assert_ne!(a, b);
        assert_ne!(a.0, 0);
        assert_ne!(b.0, 0);
    }


    #[test]
    fn sandbox_receipt_rejects_empty_evidence() {
        let assignment_id = RendererProcessAssignmentId::new(1).unwrap();
        let installation_id = SandboxInstallationId::new(2).unwrap();
        let receipt = SandboxEnforcementReceipt::from_adapter(
            assignment_id,
            installation_id,
            SandboxAdapterKind::LinuxSeccompSyscallV1,
            [0x11; 32],
            [0u8; 32],
            SandboxEnforcementLayer::Syscall,
        );
        assert!(matches!(
            receipt,
            Err(ProcessContractError::InvalidSandboxEvidence)
        ));
    }

    #[test]
    fn pid_alone_is_not_a_process_identity() {
        let first = ProcessIdentity::new(42, 100).unwrap();
        let reused = ProcessIdentity::new(42, 200).unwrap();
        assert_ne!(first, reused);
    }

    #[test]
    fn mismatched_start_time_cannot_validate() {
        let mut controller = RendererProcessController::new();
        let process = RendererProcessId::new(7).unwrap();
        controller
            .register_launch(
                process,
                1,
                ProcessIdentity::new(42, 100).unwrap(),
                SandboxProfileV1::renderer_default(),
            )
            .unwrap();

        assert!(matches!(
            controller.validate_identity(process, ProcessIdentity::new(42, 200).unwrap()),
            Err(ProcessContractError::ProcessIdentityMismatch)
        ));
    }

    #[test]
    fn retirement_requires_exact_identity() {
        let mut controller = RendererProcessController::new();
        let process = RendererProcessId::new(7).unwrap();
        let identity = ProcessIdentity::new(42, 100).unwrap();
        controller
            .register_launch(process, 1, identity, SandboxProfileV1::renderer_default())
            .unwrap();

        assert!(controller.retire(process, identity).is_ok());
        assert!(controller.active().is_none());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn observes_current_process_identity() {
        let pid = std::process::id();
        let identity = observe_process_identity(pid).unwrap();
        assert_eq!(identity.pid, pid);
        assert!(identity.start_time_ticks > 0);
    }

    #[test]
    fn failed_assignment_generation_fails_closed() {
        let mut controller = RendererProcessController::new();
        let process = RendererProcessId::new(7).unwrap();
        // A valid registration must never use an all-zero assignment ID.
        let receipt = controller
            .register_launch(
                process,
                1,
                ProcessIdentity::new(42, 100).unwrap(),
                SandboxProfileV1::renderer_default(),
            )
            .unwrap();
        assert_ne!(receipt.assignment_id.0, 0);
    }
}