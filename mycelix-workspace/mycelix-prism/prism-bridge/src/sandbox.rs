//! Linux-first renderer sandbox enforcement adapter.
//!
//! This module is intentionally a child-process bootstrap primitive, not the
//! process supervisor itself. The supervisor must launch the renderer and run
//! this bootstrap before attaching renderer IPC.
//!
//! The first enforced layer is Landlock + no_new_privs. Landlock is an
//! additive kernel access-control mechanism; it does not replace syscall
//! filtering. Seccomp is therefore a separate qualification boundary and is
//! not represented as "enforced" by this adapter until a policy-specific,
//! architecture-qualified filter is installed.

use crate::process::{SandboxAdapterKind, SandboxEnforcementReceipt, SandboxInstallationId,
    SandboxProfileV1, RendererProcessAssignmentId, ProcessContractError};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SandboxEnforcementError {
    UnsupportedPlatform,
    KernelInterfaceUnavailable,
    InvalidRuleset,
    EnforcementFailed(i32),
    IdentityGenerationFailed,
}

impl core::fmt::Display for SandboxEnforcementError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnsupportedPlatform => f.write_str("renderer sandbox unsupported on this platform"),
            Self::KernelInterfaceUnavailable => f.write_str("required Linux sandbox interface unavailable"),
            Self::InvalidRuleset => f.write_str("invalid Landlock ruleset"),
            Self::EnforcementFailed(errno) => write!(f, "renderer sandbox enforcement failed: errno {errno}"),
            Self::IdentityGenerationFailed => f.write_str("sandbox installation identity generation failed"),
        }
    }
}

impl std::error::Error for SandboxEnforcementError {}

#[cfg(target_os = "linux")]
mod linux {
    use super::*;
    use std::mem::size_of;
    use std::os::fd::RawFd;

    const LANDLOCK_CREATE_RULESET_VERSION: u32 = 1 << 0;
    const LANDLOCK_CREATE_RULESET_ERRATA: u32 = 1 << 1;
    const LANDLOCK_RULE_PATH_BENEATH: u16 = 1;
    const LANDLOCK_ACCESS_FS_EXECUTE: u64 = 1 << 0;
    const LANDLOCK_ACCESS_FS_WRITE_FILE: u64 = 1 << 1;
    const LANDLOCK_ACCESS_FS_READ_FILE: u64 = 1 << 2;
    const LANDLOCK_ACCESS_FS_READ_DIR: u64 = 1 << 3;
    const LANDLOCK_ACCESS_FS_REMOVE_DIR: u64 = 1 << 4;
    const LANDLOCK_ACCESS_FS_REMOVE_FILE: u64 = 1 << 5;
    const LANDLOCK_ACCESS_FS_MAKE_CHAR: u64 = 1 << 6;
    const LANDLOCK_ACCESS_FS_MAKE_DIR: u64 = 1 << 7;
    const LANDLOCK_ACCESS_FS_MAKE_REG: u64 = 1 << 8;
    const LANDLOCK_ACCESS_FS_MAKE_SOCK: u64 = 1 << 9;
    const LANDLOCK_ACCESS_FS_MAKE_FIFO: u64 = 1 << 10;
    const LANDLOCK_ACCESS_FS_MAKE_BLOCK: u64 = 1 << 11;
    const LANDLOCK_ACCESS_FS_MAKE_SYM: u64 = 1 << 12;
    const LANDLOCK_ACCESS_FS_REFER: u64 = 1 << 13;
    const LANDLOCK_ACCESS_FS_TRUNCATE: u64 = 1 << 14;
    const LANDLOCK_ACCESS_FS_IOCTL_DEV: u64 = 1 << 15;

    #[repr(C)]
    struct RulesetAttr {
        handled_access_fs: u64,
        _reserved: u64,
    }

    fn landlock_create_ruleset(attr: *const RulesetAttr, flags: u32) -> Result<RawFd, SandboxEnforcementError> {
        let rc = unsafe { libc::syscall(libc::SYS_landlock_create_ruleset, attr, size_of::<RulesetAttr>(), flags) };
        if rc < 0 {
            return Err(SandboxEnforcementError::EnforcementFailed(
                std::io::Error::last_os_error().raw_os_error().unwrap_or(libc::ENOSYS),
            ));
        }
        Ok(rc as RawFd)
    }

    fn set_no_new_privs() -> Result<(), SandboxEnforcementError> {
        let rc = unsafe { libc::prctl(libc::PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) };
        if rc != 0 {
            return Err(SandboxEnforcementError::EnforcementFailed(
                std::io::Error::last_os_error().raw_os_error().unwrap_or(libc::EINVAL),
            ));
        }
        Ok(())
    }

    fn restrict_self(fd: RawFd) -> Result<(), SandboxEnforcementError> {
        const LANDLOCK_RESTRICT_SELF_NO_NEW_PRIVS: u32 = 1 << 2;
        let rc = unsafe { libc::syscall(libc::SYS_landlock_restrict_self, fd, LANDLOCK_RESTRICT_SELF_NO_NEW_PRIVS) };
        if rc != 0 {
            return Err(SandboxEnforcementError::EnforcementFailed(
                std::io::Error::last_os_error().raw_os_error().unwrap_or(libc::EPERM),
            ));
        }
        Ok(())
    }

    /// Install the first real OS-enforced renderer boundary.
    ///
    /// This deliberately requires a caller-provided filesystem root. An
    /// empty/no-root policy would create a false sense of confinement because
    /// Landlock rulesets only restrict the access rights they explicitly
    /// handle. Network and syscall filtering remain separate boundaries.
    pub fn install(
        assignment_id: RendererProcessAssignmentId,
        profile: SandboxProfileV1,
        allowed_root: &std::path::Path,
    ) -> Result<SandboxEnforcementReceipt, SandboxEnforcementError> {
        if !allowed_root.is_absolute() {
            return Err(SandboxEnforcementError::InvalidRuleset);
        }

        let handled = LANDLOCK_ACCESS_FS_EXECUTE
            | LANDLOCK_ACCESS_FS_WRITE_FILE
            | LANDLOCK_ACCESS_FS_READ_FILE
            | LANDLOCK_ACCESS_FS_READ_DIR
            | LANDLOCK_ACCESS_FS_REMOVE_DIR
            | LANDLOCK_ACCESS_FS_REMOVE_FILE
            | LANDLOCK_ACCESS_FS_MAKE_CHAR
            | LANDLOCK_ACCESS_FS_MAKE_DIR
            | LANDLOCK_ACCESS_FS_MAKE_REG
            | LANDLOCK_ACCESS_FS_MAKE_SOCK
            | LANDLOCK_ACCESS_FS_MAKE_FIFO
            | LANDLOCK_ACCESS_FS_MAKE_BLOCK
            | LANDLOCK_ACCESS_FS_MAKE_SYM
            | LANDLOCK_ACCESS_FS_REFER
            | LANDLOCK_ACCESS_FS_TRUNCATE
            | LANDLOCK_ACCESS_FS_IOCTL_DEV;

        let attr = RulesetAttr { handled_access_fs: handled, _reserved: 0 };
        let fd = landlock_create_ruleset(&attr, 0)?;
        let root_fd = std::fs::File::open(allowed_root)
            .map_err(|e| SandboxEnforcementError::EnforcementFailed(e.raw_os_error().unwrap_or(libc::EACCES)))?;

        #[repr(C)]
        struct PathBeneathAttr {
            allowed_access: u64,
            parent_fd: u64,
        }
        let rule = PathBeneathAttr {
            allowed_access: handled,
            parent_fd: root_fd.as_raw_fd() as u64,
        };

        let rc = unsafe {
            libc::syscall(
                libc::SYS_landlock_add_rule,
                fd,
                LANDLOCK_RULE_PATH_BENEATH,
                &rule,
                0,
            )
        };
        if rc != 0 {
            let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(libc::EACCES);
            unsafe { libc::close(fd); }
            return Err(SandboxEnforcementError::EnforcementFailed(errno));
        }

        set_no_new_privs()?;
        restrict_self(fd)?;
        unsafe { libc::close(fd); }

        // We only report actual enforcement after restrict_self() succeeds.
        let installation_id = SandboxInstallationId::new(
            u128::from_be_bytes({
                let mut bytes = [0u8; 16];
                getrandom::fill(&mut bytes).map_err(|_| SandboxEnforcementError::IdentityGenerationFailed)?;
                bytes
            })
        ).map_err(|_| SandboxEnforcementError::IdentityGenerationFailed)?;

        Ok(SandboxEnforcementReceipt {
            assignment_id,
            installation_id,
            adapter: SandboxAdapterKind::LinuxSeccompLandlockV1,
            policy_digest: profile.policy_digest(),
            enforced: true,
        })
    }
}

#[cfg(target_os = "linux")]
pub use linux::install as install_linux;

#[cfg(not(target_os = "linux"))]
pub fn install_linux(
    _assignment_id: RendererProcessAssignmentId,
    _profile: SandboxProfileV1,
    _allowed_root: &std::path::Path,
) -> Result<SandboxEnforcementReceipt, SandboxEnforcementError> {
    Err(SandboxEnforcementError::UnsupportedPlatform)
}

/// Convert the sandbox adapter's non-secret success into the supervisor
/// receipt. This is intentionally separate from policy construction.
pub fn require_installation_id(
    value: u128,
) -> Result<SandboxInstallationId, SandboxEnforcementError> {
    SandboxInstallationId::new(value)
        .map_err(|_| SandboxEnforcementError::IdentityGenerationFailed)
}
