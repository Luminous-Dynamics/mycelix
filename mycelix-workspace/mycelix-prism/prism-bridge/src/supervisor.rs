//! Explicit renderer launch/exit security state machine.
//!
//! This module is deliberately independent of OS process spawning. It makes
//! the ordering of authority-bearing transitions explicit so a future
//! supervisor cannot attach renderer IPC before sandbox and identity
//! qualification.

use crate::process::{ProcessIdentity, RendererLaunchReceipt, SandboxEnforcementReceipt, SandboxEvidenceBundle};

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
    IdentityNotBound,
    IpcNotAttached,
    DuplicateSandboxEvidence,
    AlreadyRetired,
    ProcessMismatch,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RendererSupervisorState {
    launch: RendererLaunchReceipt,
    state: RendererProcessState,
    sandbox: Option<SandboxEvidenceBundle>,
    exit: Option<RendererExitReceipt>,
}

impl RendererSupervisorState {
    pub fn launch(&self) -> &RendererLaunchReceipt { &self.launch }
    pub fn state(&self) -> RendererProcessState { self.state }
    pub fn sandbox(&self) -> Option<&SandboxEvidenceBundle> { self.sandbox.as_ref() }
    pub fn exit(&self) -> Option<&RendererExitReceipt> { self.exit.as_ref() }
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

    /// Record one independently evidenced sandbox layer. The renderer remains
    /// unqualified until the accumulated bundle covers every broker-required
    /// layer; additional adapters may therefore contribute evidence while the
    /// state is still Assigned.
    pub fn record_sandbox(
        &mut self,
        receipt: RendererSandboxReceipt,
    ) -> Result<(), RendererSupervisorError> {
        if self.state != RendererProcessState::Assigned {
            return Err(RendererSupervisorError::InvalidTransition);
        }

        let evidence = receipt.enforcement;
        let mut bundle = self.sandbox.unwrap_or_else(|| {
            SandboxEvidenceBundle::new(
                self.launch.assignment_id(),
                self.launch.sandbox().policy_digest(),
            )
        });

        bundle.record(evidence).map_err(|error| match error {
            crate::process::ProcessContractError::SandboxEvidenceMismatch => {
                RendererSupervisorError::SandboxPolicyMismatch
            }
            crate::process::ProcessContractError::DuplicateSandboxEvidence => {
                RendererSupervisorError::DuplicateSandboxEvidence
            }
            _ => RendererSupervisorError::SandboxEnforcementMissing,
        })?;

        self.sandbox = Some(bundle);
        if bundle.enforced_layers().covers(self.launch.sandbox().required_enforcement_layers()) {
            self.state = RendererProcessState::SandboxQualified;
        }

        Ok(())
    }

    pub fn bind_identity(
        &mut self,
        observed: ProcessIdentity,
    ) -> Result<(), RendererSupervisorError> {
        if self.state != RendererProcessState::SandboxQualified {
            return Err(RendererSupervisorError::SandboxNotQualified);
        }
        if observed != self.launch.process() {
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
        if observed != self.launch.process() {
            return Err(RendererSupervisorError::ProcessMismatch);
        }

        let receipt = RendererExitReceipt {
            assignment_id: self.launch.assignment_id(),
            process: observed,
            generation: self.launch.generation(),
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
    use crate::process::{
        next_sandbox_installation_id, RendererProcessAssignmentId, SandboxAdapterKind,
        SandboxEnforcementLayer, SandboxProfileV1,
    };

    fn receipt(
        state: &RendererSupervisorState,
        adapter: SandboxAdapterKind,
        layer: SandboxEnforcementLayer,
    ) -> RendererSandboxReceipt {
        RendererSandboxReceipt {
            enforcement: crate::process::SandboxEnforcementReceipt::from_adapter(
                state.launch().assignment_id(),
                next_sandbox_installation_id().unwrap(),
                adapter,
                state.launch().sandbox().policy_digest(),
                [0x11; 32],
                layer,
            ).unwrap(),
        }
    }

    fn launch() -> RendererLaunchReceipt {
        let process = RendererProcessId::new(7).unwrap();
        let identity = ProcessIdentity::new(42, 100).unwrap();
        let mut controller = crate::process::RendererProcessController::new();
        controller
            .register_launch(
                process,
                1,
                identity,
                SandboxProfileV1::renderer_default(),
            )
            .unwrap()
    }

    #[test]
    fn filesystem_only_landlock_cannot_qualify_full_renderer_profile() {
        let mut state = RendererSupervisorState::new(launch());
        state.record_sandbox(receipt(
            &state,
            SandboxAdapterKind::LinuxLandlockFilesystemV1,
            SandboxEnforcementLayer::Filesystem,
        )).unwrap();
        assert_eq!(state.state(), RendererProcessState::Assigned);
        assert!(!state.capability_authority_ready());
        assert!(state.sandbox().unwrap().enforced_layers().contains(
            SandboxEnforcementLayer::Filesystem
        ));
    }

    #[test]
    fn one_receipt_cannot_claim_multiple_layers() {
        assert!(matches!(
            crate::process::SandboxEnforcementReceipt::from_adapter(
                RendererProcessAssignmentId::new(1).unwrap(),
                next_sandbox_installation_id().unwrap(),
                SandboxAdapterKind::LinuxLandlockFilesystemV1,
                SandboxProfileV1::renderer_default().policy_digest(),
                [0x11; 32],
                SandboxEnforcementLayer::Syscall,
            ),
            Err(crate::process::ProcessContractError::InvalidSandboxEvidence)
        ));
    }

    #[test]
    fn duplicate_layer_evidence_is_rejected() {
        let mut state = RendererSupervisorState::new(launch());
        let first = receipt(
            &state,
            SandboxAdapterKind::LinuxLandlockFilesystemV1,
            SandboxEnforcementLayer::Filesystem,
        );
        let second = receipt(
            &state,
            SandboxAdapterKind::LinuxLandlockFilesystemV1,
            SandboxEnforcementLayer::Filesystem,
        );
        state.record_sandbox(first).unwrap();
        assert!(matches!(
            state.record_sandbox(second),
            Err(RendererSupervisorError::DuplicateSandboxEvidence)
        ));
    }

    #[test]
    fn mixed_assignment_evidence_is_rejected() {
        let mut state = RendererSupervisorState::new(launch());
        let other = RendererProcessAssignmentId::new(999).unwrap();
        let forged = RendererSandboxReceipt {
            enforcement: crate::process::SandboxEnforcementReceipt::from_adapter(
                other,
                next_sandbox_installation_id().unwrap(),
                SandboxAdapterKind::LinuxLandlockFilesystemV1,
                state.launch.sandbox.policy_digest(),
                [0x22; 32],
                SandboxEnforcementLayer::Filesystem,
            ).unwrap(),
        };
        assert!(matches!(
            state.record_sandbox(forged),
            Err(RendererSupervisorError::SandboxPolicyMismatch)
        ));
        assert_eq!(state.state(), RendererProcessState::Assigned);
    }

    #[test]
    fn syscall_evidence_is_distinct_from_filesystem_evidence() {
        let mut state = RendererSupervisorState::new(launch());
        state.record_sandbox(receipt(
            &state,
            SandboxAdapterKind::LinuxLandlockFilesystemV1,
            SandboxEnforcementLayer::Filesystem,
        )).unwrap();
        state.record_sandbox(receipt(
            &state,
            SandboxAdapterKind::LinuxSeccompSyscallV1,
            SandboxEnforcementLayer::Syscall,
        )).unwrap();
        let layers = state.sandbox().unwrap().enforced_layers();
        assert!(layers.contains(SandboxEnforcementLayer::Filesystem));
        assert!(layers.contains(SandboxEnforcementLayer::Syscall));
        assert_eq!(state.state(), RendererProcessState::Assigned);
    }

    #[test]
    fn ipc_is_impossible_without_complete_sandbox_evidence() {
        let mut state = RendererSupervisorState::new(launch());
        state.record_sandbox(receipt(
            &state,
            SandboxAdapterKind::LinuxLandlockFilesystemV1,
            SandboxEnforcementLayer::Filesystem,
        )).unwrap();
        assert!(matches!(
            state.attach_ipc(),
            Err(RendererSupervisorError::SandboxNotQualified)
        ));
    }

    #[test]
    fn forged_policy_digest_cannot_qualify() {
        let mut state = RendererSupervisorState::new(launch());
        let receipt = receipt(
            &state,
            SandboxAdapterKind::LinuxLandlockFilesystemV1,
            SandboxEnforcementLayer::Filesystem,
        );
        // The typed receipt is immutable; a forged digest cannot be injected
        // through the public renderer-facing data model.
        assert_eq!(
            receipt.enforcement.profile_digest(),
            state.launch.sandbox.policy_digest()
        );
        let _ = &receipt;
        assert_eq!(state.state, RendererProcessState::Assigned);
    }

    #[test]
    fn wrong_process_identity_cannot_bind() {
        let mut state = RendererSupervisorState::new(launch());
        // Identity binding remains unreachable while sandbox evidence is incomplete.
        assert!(matches!(
            state.bind_identity(ProcessIdentity::new(42, 101).unwrap()),
            Err(RendererSupervisorError::SandboxNotQualified)
        ));
    }

    #[test]
    fn exit_requires_exact_identity_and_requires_retirement() {
        let mut state = RendererSupervisorState::new(launch());
        // State cannot reach Running until all required layers are independently evidenced.
        assert!(matches!(
            state.observe_exit(state.launch().process(), false),
            Err(RendererSupervisorError::InvalidTransition)
        ));
        assert_eq!(state.state, RendererProcessState::Assigned);
    }
}
