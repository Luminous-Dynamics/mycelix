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

use crate::process::{
    SandboxAdapterKind, SandboxEnforcementReceipt, SandboxInstallationId, SandboxProfileV1,
    RendererProcessAssignmentId, SandboxEnforcementLayer,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SandboxEnforcementError {
    UnsupportedPlatform,
    KernelInterfaceUnavailable,
    InvalidRuleset,
    EnforcementFailed(i32),
    LandlockAbiTooOld(u32),
    IdentityGenerationFailed,
    InvalidAssignmentId,
}

impl core::fmt::Display for SandboxEnforcementError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnsupportedPlatform => f.write_str("renderer sandbox unsupported on this platform"),
            Self::KernelInterfaceUnavailable => f.write_str("required Linux sandbox interface unavailable"),
            Self::InvalidRuleset => f.write_str("invalid Landlock ruleset"),
            Self::EnforcementFailed(errno) => write!(f, "renderer sandbox enforcement failed: errno {errno}"),
            Self::LandlockAbiTooOld(abi) => write!(f, "Landlock ABI {abi} lacks renderer thread-synchronization support"),
            Self::IdentityGenerationFailed => f.write_str("sandbox installation identity generation failed"),
            Self::InvalidAssignmentId => f.write_str("renderer process assignment id must be non-zero"),
        }
    }
}

impl std::error::Error for SandboxEnforcementError {}

#[cfg(target_os = "linux")]
mod linux {
    use super::*;
    use std::mem::size_of;
    use std::ffi::CString;
    use std::os::fd::{AsRawFd, FromRawFd, RawFd};
    use std::os::unix::ffi::OsStrExt;

    const LANDLOCK_CREATE_RULESET_VERSION: u32 = 1 << 0;
    const LANDLOCK_MIN_ABI_FOR_TSYNC: u32 = 8;
    const LANDLOCK_RESTRICT_SELF_TSYNC: u32 = 1 << 3;
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

    fn landlock_abi_version() -> Result<u32, SandboxEnforcementError> {
        let rc = unsafe {
            libc::syscall(
                libc::SYS_landlock_create_ruleset,
                std::ptr::null::<RulesetAttr>(),
                0usize,
                LANDLOCK_CREATE_RULESET_VERSION,
            )
        };
        if rc < 0 {
            return Err(SandboxEnforcementError::EnforcementFailed(
                std::io::Error::last_os_error().raw_os_error().unwrap_or(libc::ENOSYS),
            ));
        }
        Ok(rc as u32)
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
        let rc = unsafe { libc::syscall(libc::SYS_landlock_restrict_self, fd, LANDLOCK_RESTRICT_SELF_TSYNC) };
        if rc != 0 {
            return Err(SandboxEnforcementError::EnforcementFailed(
                std::io::Error::last_os_error().raw_os_error().unwrap_or(libc::EPERM),
            ));
        }
        Ok(())
    }

    fn filesystem_evidence_digest(abi: u32, handled_access_fs: u64, allowed_root: &std::path::Path) -> [u8; 32] {
        let root = allowed_root.to_string_lossy();
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"PRISM-LANDLOCK-FILESYSTEM-EVIDENCE-V2");
        hasher.update(&abi.to_le_bytes());
        hasher.update(&handled_access_fs.to_le_bytes());
        hasher.update(&(root.len() as u64).to_le_bytes());
        hasher.update(root.as_bytes());
        *hasher.finalize().as_bytes()
    }

    /// Install the first real OS-enforced filesystem boundary.
    ///
    /// This deliberately reports only the filesystem layer. It does NOT
    /// claim that the complete renderer profile (network, child-process,
    /// syscall, device policy) has been enforced. The supervisor therefore
    /// must not treat this receipt as sufficient for capability authority.
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
        if assignment_id.0 == 0 {
            return Err(SandboxEnforcementError::InvalidAssignmentId);
        }
        if !allowed_root.is_absolute() {
            return Err(SandboxEnforcementError::InvalidRuleset);
        }

        let abi = landlock_abi_version()?;
        if abi < LANDLOCK_MIN_ABI_FOR_TSYNC {
            return Err(SandboxEnforcementError::LandlockAbiTooOld(abi));
        }

        // Mint the installation identity before irreversible restriction.
        // The filesystem policy may be followed by stricter layers that deny
        // runtime services such as getrandom(2); evidence construction must
        // never fail after enforcement has already become irreversible.
        let mut installation_bytes = [0u8; 16];
        getrandom::fill(&mut installation_bytes)
            .map_err(|_| SandboxEnforcementError::IdentityGenerationFailed)?;
        let installation_id = SandboxInstallationId::new(
            u128::from_be_bytes(installation_bytes)
        ).map_err(|_| SandboxEnforcementError::IdentityGenerationFailed)?;

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
        // Landlock identifies PATH_BENEATH roots by file descriptor. Use
        // O_PATH|O_CLOEXEC as recommended by the kernel interface so the
        // qualification root is identified without requiring read access and
        // without leaking the descriptor across exec.
        let root_path = CString::new(allowed_root.as_os_str().as_bytes())
            .map_err(|_| SandboxEnforcementError::InvalidRuleset)?;
        let root_fd_raw = unsafe {
            libc::open(
                root_path.as_ptr(),
                libc::O_PATH | libc::O_CLOEXEC,
            )
        };
        if root_fd_raw < 0 {
            return Err(SandboxEnforcementError::EnforcementFailed(
                std::io::Error::last_os_error().raw_os_error().unwrap_or(libc::EACCES),
            ));
        }
        let root_fd = unsafe { std::fs::File::from_raw_fd(root_fd_raw) };

        #[repr(C, packed)]
        struct PathBeneathAttr {
            allowed_access: u64,
            parent_fd: i32,
        }
        let rule = PathBeneathAttr {
            allowed_access: handled,
            parent_fd: root_fd.as_raw_fd(),
        };

        let rc = unsafe {
            libc::syscall(
                libc::SYS_landlock_add_rule,
                fd,
                LANDLOCK_RULE_PATH_BENEATH as libc::c_uint,
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
        SandboxEnforcementReceipt::from_adapter(
            assignment_id,
            installation_id,
            SandboxAdapterKind::LinuxLandlockFilesystemV1,
            profile.policy_digest(),
            filesystem_evidence_digest(abi, handled, allowed_root),
            SandboxEnforcementLayer::Filesystem,
        ).map_err(|_| SandboxEnforcementError::InvalidRuleset)
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
