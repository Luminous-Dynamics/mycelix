//! Explicit renderer launch/exit security state machine.
//!
//! This module is deliberately independent of OS process spawning. It makes
//! the ordering of authority-bearing transitions explicit so a future
//! supervisor cannot attach renderer IPC before sandbox and identity
//! qualification.

use crate::process::{ProcessIdentity, RendererLaunchReceipt, SandboxEnforcementReceipt, SandboxEnforcementLayer, SandboxEnforcementSet, SandboxProfileV1};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RendererProcessState {
    Assigned,
    SandboxQualified,
    IdentityBound,
    IpcAttached,
    Running,
    ExitObserved,
    Retired,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RendererSandboxReceipt {
    pub enforcement: SandboxEnforcementReceipt,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RendererExitReceipt {
    pub assignment_id: crate::process::RendererProcessAssignmentId,
    pub process: ProcessIdentity,
    pub generation: u64,
    pub expected: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RendererSupervisorError {
    InvalidTransition,
    SandboxNotQualified,
    SandboxEnforcementMissing,
    SandboxPolicyMismatch,
    SandboxAssignmentMismatch,
    SandboxInstallationInvalid,
    IdentityNotBound,
    IpcNotAttached,
    AlreadyRetired,
    ProcessMismatch,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RendererSupervisorState {
    pub launch: RendererLaunchReceipt,
    pub state: RendererProcessState,
    pub sandbox: Option<RendererSandboxReceipt>,
    pub exit: Option<RendererExitReceipt>,
}

impl RendererSupervisorState {
    pub fn new(launch: RendererLaunchReceipt) -> Self {
        Self {
            launch,
            state: RendererProcessState::Assigned,
            sandbox: None,
            exit: None,
        }
    }

    /// A capability-bearing renderer cannot progress until the OS adapter has
    /// positively reported that the required sandbox profile was installed.
    pub fn record_sandbox(
        &mut self,
        receipt: RendererSandboxReceipt,
    ) -> Result<(), RendererSupervisorError> {
        if self.state != RendererProcessState::Assigned {
            return Err(RendererSupervisorError::InvalidTransition);
        }
        let evidence = receipt.enforcement;
        if evidence.assignment_id != self.launch.assignment_id {
            return Err(RendererSupervisorError::SandboxAssignmentMismatch);
        }
        if evidence.installation_id.0 == 0 {
            return Err(RendererSupervisorError::SandboxInstallationInvalid);
        }
        if !evidence.enforced {
            return Err(RendererSupervisorError::SandboxEnforcementMissing);
        }
        if evidence.policy_digest != self.launch.sandbox.policy_digest() {
            return Err(RendererSupervisorError::SandboxPolicyMismatch);
        }
        if matches!(evidence.adapter, crate::process::SandboxAdapterKind::UnsupportedPlatform) {
            return Err(RendererSupervisorError::SandboxEnforcementMissing);
        }
        if !evidence.enforced_layers.covers(self.launch.sandbox.required_enforcement_layers()) {
            return Err(RendererSupervisorError::SandboxEnforcementMissing);
        }
        self.sandbox = Some(receipt);
        self.state = RendererProcessState::SandboxQualified;
        Ok(())
    }

    pub fn bind_identity(
        &mut self,
        observed: ProcessIdentity,
    ) -> Result<(), RendererSupervisorError> {
        if self.state != RendererProcessState::SandboxQualified {
            return Err(RendererSupervisorError::SandboxNotQualified);
        }
        if observed != self.launch.process {
            return Err(RendererSupervisorError::ProcessMismatch);
        }
        self.state = RendererProcessState::IdentityBound;
        Ok(())
    }

    /// IPC may attach only after sandbox qualification and identity binding.
    pub fn attach_ipc(&mut self) -> Result<(), RendererSupervisorError> {
        if self.state != RendererProcessState::IdentityBound {
            return Err(match self.state {
                RendererProcessState::Assigned => RendererSupervisorError::SandboxNotQualified,
                RendererProcessState::SandboxQualified => RendererSupervisorError::IdentityNotBound,
                _ => RendererSupervisorError::InvalidTransition,
            });
        }
        self.state = RendererProcessState::IpcAttached;
        Ok(())
    }

    pub fn mark_running(&mut self) -> Result<(), RendererSupervisorError> {
        if self.state != RendererProcessState::IpcAttached {
            return Err(RendererSupervisorError::IpcNotAttached);
        }
        self.state = RendererProcessState::Running;
        Ok(())
    }

    /// Exit is a security transition, not merely process bookkeeping.
    pub fn observe_exit(
        &mut self,
        observed: ProcessIdentity,
        expected: bool,
    ) -> Result<RendererExitReceipt, RendererSupervisorError> {
        if !matches!(
            self.state,
            RendererProcessState::IpcAttached | RendererProcessState::Running
        ) {
            return Err(RendererSupervisorError::InvalidTransition);
        }
        if observed != self.launch.process {
            return Err(RendererSupervisorError::ProcessMismatch);
        }

        let receipt = RendererExitReceipt {
            assignment_id: self.launch.assignment_id,
            process: observed,
            generation: self.launch.generation,
            expected,
        };
        self.exit = Some(receipt);
        self.state = RendererProcessState::ExitObserved;
        Ok(receipt)
    }

    /// Retirement is intentionally separate from exit observation so the
    /// security controller can revoke sessions/grants before assignment state
    /// is finally discarded.
    pub fn retire(&mut self) -> Result<(), RendererSupervisorError> {
        if self.state != RendererProcessState::ExitObserved {
            return Err(RendererSupervisorError::InvalidTransition);
        }
        self.state = RendererProcessState::Retired;
        Ok(())
    }

    pub fn capability_authority_ready(&self) -> bool {
        self.state == RendererProcessState::Running
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::identity::RendererProcessId;
    use crate::process::{next_sandbox_installation_id, SandboxAdapterKind};

    fn full_receipt(state: &RendererSupervisorState) -> RendererSandboxReceipt {
        RendererSandboxReceipt {
            enforcement: SandboxEnforcementReceipt {
                assignment_id: state.launch.assignment_id,
                installation_id: next_sandbox_installation_id().unwrap(),
                adapter: SandboxAdapterKind::LinuxLandlockFilesystemV1,
                policy_digest: state.launch.sandbox.policy_digest(),
                enforced_layers: state.launch.sandbox.required_enforcement_layers(),
                enforced: true,
            },
        }
    }

    fn filesystem_only_receipt(state: &RendererSupervisorState) -> RendererSandboxReceipt {
        RendererSandboxReceipt {
            enforcement: SandboxEnforcementReceipt {
                assignment_id: state.launch.assignment_id,
                installation_id: next_sandbox_installation_id().unwrap(),
                adapter: SandboxAdapterKind::LinuxLandlockFilesystemV1,
                policy_digest: state.launch.sandbox.policy_digest(),
                enforced_layers: SandboxEnforcementSet::from_layer(SandboxEnforcementLayer::Filesystem),
                enforced: true,
            },
        }
    }

    fn launch() -> RendererLaunchReceipt {
        let process = RendererProcessId::new(7).unwrap();
        let identity = ProcessIdentity::new(42, 100).unwrap();
        let mut controller = crate::process::RendererProcessController::new();
        controller.register_launch(process, 1, identity, SandboxProfileV1::renderer_default()).unwrap()
    }

    #[test]
    fn filesystem_only_landlock_cannot_qualify_full_renderer_profile() {
        let mut state = RendererSupervisorState::new(launch());
        assert!(matches!(state.record_sandbox(filesystem_only_receipt(&state)), Err(RendererSupervisorError::SandboxEnforcementMissing)));
        assert_eq!(state.state, RendererProcessState::Assigned);
        assert!(!state.capability_authority_ready());
    }

    #[test]
    fn ipc_is_impossible_before_full_sandbox_and_identity() {
        let mut state = RendererSupervisorState::new(launch());
        assert!(matches!(state.attach_ipc(), Err(RendererSupervisorError::SandboxNotQualified)));
        state.record_sandbox(full_receipt(&state)).unwrap();
        assert!(matches!(state.attach_ipc(), Err(RendererSupervisorError::IdentityNotBound)));
        state.bind_identity(state.launch.process).unwrap();
        state.attach_ipc().unwrap();
        state.mark_running().unwrap();
        assert!(state.capability_authority_ready());
    }

    #[test]
    fn forged_policy_digest_cannot_qualify() {
        let mut state = RendererSupervisorState::new(launch());
        let mut receipt = full_receipt(&state);
        receipt.enforcement.policy_digest[0] ^= 1;
        assert!(matches!(state.record_sandbox(receipt), Err(RendererSupervisorError::SandboxPolicyMismatch)));
    }

    #[test]
    fn wrong_process_identity_cannot_bind() {
        let mut state = RendererSupervisorState::new(launch());
        state.record_sandbox(full_receipt(&state)).unwrap();
        assert!(matches!(state.bind_identity(ProcessIdentity::new(42, 101).unwrap()), Err(RendererSupervisorError::ProcessMismatch)));
    }

    #[test]
    fn exit_requires_exact_identity_and_requires_retirement() {
        let mut state = RendererSupervisorState::new(launch());
        state.record_sandbox(full_receipt(&state)).unwrap();
        state.bind_identity(state.launch.process).unwrap();
        state.attach_ipc().unwrap();
        state.mark_running().unwrap();
        let receipt = state.observe_exit(state.launch.process, false).unwrap();
        assert!(!receipt.expected);
        assert_eq!(state.state, RendererProcessState::ExitObserved);
        assert!(!state.capability_authority_ready());
        state.retire().unwrap();
        assert_eq!(state.state, RendererProcessState::Retired);
    }
}
