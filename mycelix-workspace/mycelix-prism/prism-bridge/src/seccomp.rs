//! Linux seccomp-BPF renderer syscall enforcement.
//!
//! This adapter intentionally accepts an explicit, architecture-qualified
//! syscall allowlist. It does not invent a permissive "default renderer"
//! allowlist: a real renderer policy must be derived from qualified runtime
//! evidence and then committed by its digest.
//!
//! Security invariants:
//! - architecture is checked before syscall number;
//! - empty/oversized/duplicate lists fail closed;
//! - the filter defaults to EPERM;
//! - ptrace-style tracing is never introduced by this adapter;
//! - no_new_privs is required before unprivileged filter installation;
//! - a successful receipt proves only the Syscall layer, not a complete sandbox.

use crate::process::{
    SandboxAdapterKind, SandboxEnforcementLayer, SandboxEnforcementReceipt,
    SandboxInstallationId, SandboxProfileV1, RendererProcessAssignmentId,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u32)]
pub enum SeccompArchitecture {
    X86_64 = 0xc000003e,
    Aarch64 = 0xc00000b7,
    Riscv64 = 0xc00000f3,
}

impl SeccompArchitecture {
    pub const fn current() -> Option<Self> {
        #[cfg(target_arch = "x86_64")]
        { Some(Self::X86_64) }
        #[cfg(target_arch = "aarch64")]
        { Some(Self::Aarch64) }
        #[cfg(target_arch = "riscv64")]
        { Some(Self::Riscv64) }
        #[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64", target_arch = "riscv64")))]
        { None }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SeccompSyscallPolicyV1 {
    pub architecture: SeccompArchitecture,
    pub allowed_syscalls: Vec<i64>,
}

impl SeccompSyscallPolicyV1 {
    pub fn new(
        architecture: SeccompArchitecture,
        mut allowed_syscalls: Vec<i64>,
    ) -> Result<Self, SeccompError> {
        const MAX_SYSCALLS: usize = 256;
        if allowed_syscalls.is_empty() || allowed_syscalls.len() > MAX_SYSCALLS {
            return Err(SeccompError::InvalidPolicy);
        }
        if allowed_syscalls.iter().any(|n| *n < 0 || *n > u32::MAX as i64) {
            return Err(SeccompError::InvalidPolicy);
        }
        allowed_syscalls.sort_unstable();
        if allowed_syscalls.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(SeccompError::DuplicateSyscall);
        }
        Ok(Self { architecture, allowed_syscalls })
    }

    pub fn digest(&self) -> [u8; 32] {
        let mut bytes = Vec::with_capacity(4 + self.allowed_syscalls.len() * 4);
        bytes.extend_from_slice(&(self.architecture as u32).to_le_bytes());
        for syscall in &self.allowed_syscalls {
            bytes.extend_from_slice(&(*syscall as u32).to_le_bytes());
        }
        *blake3::hash(&bytes).as_bytes()
    }

    pub fn allows(&self, syscall: i64) -> bool {
        self.allowed_syscalls.binary_search(&syscall).is_ok()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeccompError {
    UnsupportedPlatform,
    ArchitectureMismatch,
    InvalidPolicy,
    DuplicateSyscall,
    FilterTooLarge,
    InstallationFailed(i32),
    IdentityGenerationFailed,
}

impl core::fmt::Display for SeccompError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnsupportedPlatform => f.write_str("seccomp unsupported on this platform"),
            Self::ArchitectureMismatch => f.write_str("seccomp policy architecture does not match the running architecture"),
            Self::InvalidPolicy => f.write_str("invalid seccomp syscall policy"),
            Self::DuplicateSyscall => f.write_str("seccomp syscall policy contains a duplicate"),
            Self::FilterTooLarge => f.write_str("seccomp BPF filter exceeds the bounded instruction budget"),
            Self::InstallationFailed(errno) => write!(f, "seccomp installation failed: errno {errno}"),
            Self::IdentityGenerationFailed => f.write_str("seccomp installation identity generation failed"),
        }
    }
}

impl std::error::Error for SeccompError {}

#[cfg(target_os = "linux")]
mod linux {
    use super::*;

    #[repr(C)]
    #[derive(Clone, Copy)]
    struct SockFilter {
        code: u16,
        jt: u8,
        jf: u8,
        k: u32,
    }

    #[repr(C)]
    struct SockFprog {
        len: u16,
        filter: *const SockFilter,
    }

    const BPF_LD: u16 = 0x00;
    const BPF_W: u16 = 0x00;
    const BPF_ABS: u16 = 0x20;
    const BPF_JMP: u16 = 0x05;
    const BPF_JEQ: u16 = 0x10;
    const BPF_K: u16 = 0x00;
    const BPF_RET: u16 = 0x06;

    const SECCOMP_MODE_FILTER: libc::c_int = 2;
    const SECCOMP_RET_KILL_PROCESS: u32 = 0x8000_0000;
    const SECCOMP_RET_ERRNO: u32 = 0x0005_0000;
    const SECCOMP_RET_ALLOW: u32 = 0x7fff_0000;

    const SECCOMP_DATA_NR_OFFSET: u32 = 0;
    const SECCOMP_DATA_ARCH_OFFSET: u32 = 4;

    fn stmt(code: u16, k: u32) -> SockFilter {
        SockFilter { code, jt: 0, jf: 0, k }
    }

    fn jump_eq(k: u32, jt: u8, jf: u8) -> SockFilter {
        SockFilter { code: BPF_JMP | BPF_JEQ | BPF_K, jt, jf, k }
    }

    pub(crate) fn compile_filter(policy: &SeccompSyscallPolicyV1) -> Result<Vec<SockFilter>, SeccompError> {
        if SeccompArchitecture::current() != Some(policy.architecture) {
            return Err(SeccompError::ArchitectureMismatch);
        }

        // Architecture check plus two instructions per allowlisted syscall
        // (match + ALLOW), followed by the bounded default-deny action.
        // Keeping the match/ALLOW pair adjacent avoids jump-offset overflow
        // while making the control flow mechanically auditable.
        let instruction_count = 5usize
            .saturating_add(policy.allowed_syscalls.len().saturating_mul(2));
        if instruction_count > 4096 || instruction_count > u16::MAX as usize {
            return Err(SeccompError::FilterTooLarge);
        }

        let mut filter = Vec::with_capacity(instruction_count);
        filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, SECCOMP_DATA_ARCH_OFFSET));
        filter.push(jump_eq(policy.architecture as u32, 1, 0));
        filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_KILL_PROCESS));
        filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, SECCOMP_DATA_NR_OFFSET));

        for syscall in &policy.allowed_syscalls {
            // Match -> next instruction is ALLOW; mismatch skips that ALLOW
            // and continues with the next syscall comparison.
            filter.push(jump_eq(*syscall as u32, 0, 1));
            filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_ALLOW));
        }

        filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | libc::EPERM as u32));

        Ok(filter)
    }

    /// Install an explicit architecture-qualified syscall allowlist into the
    /// current process. This call is intended for the renderer bootstrap after
    /// process identity is established and before renderer capability IPC.
    pub fn install(
        assignment_id: RendererProcessAssignmentId,
        profile: SandboxProfileV1,
        policy: &SeccompSyscallPolicyV1,
    ) -> Result<SandboxEnforcementReceipt, SeccompError> {
        let mut filter = compile_filter(policy)?;

        let rc = unsafe { libc::prctl(libc::PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) };
        if rc != 0 {
            return Err(SeccompError::InstallationFailed(
                std::io::Error::last_os_error().raw_os_error().unwrap_or(libc::EPERM),
            ));
        }

        let program = SockFprog {
            len: u16::try_from(filter.len()).map_err(|_| SeccompError::FilterTooLarge)?,
            filter: filter.as_mut_ptr(),
        };

        let rc = unsafe {
            libc::prctl(
                libc::PR_SET_SECCOMP,
                SECCOMP_MODE_FILTER,
                &program as *const SockFprog,
                0,
                0,
            )
        };
        if rc != 0 {
            return Err(SeccompError::InstallationFailed(
                std::io::Error::last_os_error().raw_os_error().unwrap_or(libc::EPERM),
            ));
        }

        let mut bytes = [0u8; 16];
        getrandom::fill(&mut bytes).map_err(|_| SeccompError::IdentityGenerationFailed)?;
        let installation_id = SandboxInstallationId::new(u128::from_be_bytes(bytes))
            .map_err(|_| SeccompError::IdentityGenerationFailed)?;

        SandboxEnforcementReceipt::from_adapter(
            assignment_id,
            installation_id,
            SandboxAdapterKind::LinuxSeccompSyscallV1,
            profile.policy_digest(),
            policy.digest(),
            SandboxEnforcementLayer::Syscall,
        ).map_err(|_| SeccompError::InvalidPolicy)
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn architecture_guard_is_present_before_syscall_allowlist() {
            let arch = SeccompArchitecture::current().unwrap();
            let policy = SeccompSyscallPolicyV1::new(arch, vec![libc::SYS_read, libc::SYS_write]).unwrap();
            let filter = compile_filter(&policy).unwrap();
            assert_eq!(filter[0].code, BPF_LD | BPF_W | BPF_ABS);
            assert_eq!(filter[0].k, SECCOMP_DATA_ARCH_OFFSET);
            assert_eq!(filter[1].k, arch as u32);
            assert_eq!(filter[2].k, SECCOMP_RET_KILL_PROCESS);
        }

        #[test]
        fn policy_is_sorted_and_deduplicated() {
            let arch = SeccompArchitecture::current().unwrap();
            let policy = SeccompSyscallPolicyV1::new(arch, vec![9, 3, 7]).unwrap();
            assert_eq!(policy.allowed_syscalls, vec![3, 7, 9]);
            assert!(SeccompSyscallPolicyV1::new(arch, vec![3, 3]).is_err());
        }

        #[test]
        fn each_allowlisted_syscall_has_a_reachable_allow_action() {
            let arch = SeccompArchitecture::current().unwrap();
            let policy = SeccompSyscallPolicyV1::new(arch, vec![libc::SYS_read, libc::SYS_write]).unwrap();
            let filter = compile_filter(&policy).unwrap();

            // 0: load arch, 1: arch match, 2: kill on mismatch, 3: load nr.
            // Each syscall then has JEQ -> ALLOW; mismatch skips exactly one
            // instruction and reaches the next comparison.
            assert_eq!(filter[4].jt, 0);
            assert_eq!(filter[4].jf, 1);
            assert_eq!(filter[5].k, SECCOMP_RET_ALLOW);
            assert_eq!(filter[6].jt, 0);
            assert_eq!(filter[6].jf, 1);
            assert_eq!(filter[7].k, SECCOMP_RET_ALLOW);
            assert_eq!(filter[8].k, SECCOMP_RET_ERRNO | libc::EPERM as u32);
            assert_eq!(filter.len(), 9);
        }

        #[test]
        fn wrong_architecture_fails_closed() {
            let arch = SeccompArchitecture::current().unwrap();
            let other = match arch {
                SeccompArchitecture::X86_64 => SeccompArchitecture::Aarch64,
                _ => SeccompArchitecture::X86_64,
            };
            let policy = SeccompSyscallPolicyV1::new(other, vec![1]).unwrap();
            assert!(matches!(compile_filter(&policy), Err(SeccompError::ArchitectureMismatch)));
        }

        #[test]
        fn empty_policy_is_rejected() {
            let arch = SeccompArchitecture::current().unwrap();
            assert!(matches!(
                SeccompSyscallPolicyV1::new(arch, vec![]),
                Err(SeccompError::InvalidPolicy)
            ));
        }

        #[test]
        fn digest_changes_with_policy() {
            let arch = SeccompArchitecture::current().unwrap();
            let a = SeccompSyscallPolicyV1::new(arch, vec![1, 2]).unwrap();
            let b = SeccompSyscallPolicyV1::new(arch, vec![1, 3]).unwrap();
            assert_ne!(a.digest(), b.digest());
        }
    }
}

#[cfg(target_os = "linux")]
pub use linux::install;

#[cfg(not(target_os = "linux"))]
pub fn install(
    _assignment_id: RendererProcessAssignmentId,
    _profile: SandboxProfileV1,
    _policy: &SeccompSyscallPolicyV1,
) -> Result<SandboxEnforcementReceipt, SeccompError> {
    Err(SeccompError::UnsupportedPlatform)
}

