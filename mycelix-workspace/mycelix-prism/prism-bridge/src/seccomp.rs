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
    architecture: SeccompArchitecture,
    allowed_syscalls: Vec<i64>,
}

impl SeccompSyscallPolicyV1 {
    pub fn new(
        architecture: SeccompArchitecture,
        mut allowed_syscalls: Vec<i64>,
    ) -> Result<Self, SeccompError> {
        const MAX_SYSCALLS: usize = 256;
        const X32_SYSCALL_BIT: i64 = 0x4000_0000;
        if allowed_syscalls.is_empty() || allowed_syscalls.len() > MAX_SYSCALLS {
            return Err(SeccompError::InvalidPolicy);
        }
        // seccomp_data.nr is a signed 32-bit syscall number. Values outside
        // that representable domain are not meaningful policy inputs and
        // should never reach BPF generation.
        if allowed_syscalls.iter().any(|n| *n < 0 || *n > i32::MAX as i64) {
            return Err(SeccompError::InvalidPolicy);
        }
        // x86-64 and x32 share AUDIT_ARCH_X86_64. Do not permit a policy to
        // explicitly allow an x32-tagged syscall number, because the
        // architecture field alone cannot distinguish the two ABIs.
        if architecture == SeccompArchitecture::X86_64
            && allowed_syscalls
                .iter()
                .any(|n| (*n & X32_SYSCALL_BIT) != 0)
        {
            return Err(SeccompError::InvalidPolicy);
        }
        allowed_syscalls.sort_unstable();
        if allowed_syscalls.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(SeccompError::DuplicateSyscall);
        }
        Ok(Self { architecture, allowed_syscalls })
    }

    pub const fn architecture(&self) -> SeccompArchitecture {
        self.architecture
    }

    pub fn allowed_syscalls(&self) -> &[i64] {
        &self.allowed_syscalls
    }

    pub fn digest(&self) -> [u8; 32] {
        // Version and length-prefix the canonical serialization so future
        // policy-format changes cannot silently reuse an older commitment
        // domain, and the framing remains mechanically unambiguous.
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"PRISM-SECCOMP-SYSCALL-POLICY-V1");
        hasher.update(&(self.architecture as u32).to_le_bytes());
        hasher.update(&(self.allowed_syscalls.len() as u32).to_le_bytes());
        for syscall in &self.allowed_syscalls {
            hasher.update(&(*syscall as u32).to_le_bytes());
        }
        *hasher.finalize().as_bytes()
    }

    pub fn allows(&self, syscall: i64) -> bool {
        self.allowed_syscalls.binary_search(&syscall).is_ok()
    }

    /// Validate the stricter policy boundary required before this renderer
    /// adapter can install a policy. Generic policy construction remains
    /// flexible, but renderer qualification never admits direct tracing,
    /// cross-process memory, process creation/image replacement, or namespace
    /// mutation primitives.
    #[cfg(target_os = "linux")]
    pub fn validate_renderer_policy(&self) -> Result<(), SeccompError> {
        const FORBIDDEN_RENDERER_SYSCALLS: &[i64] = &[
            libc::SYS_ptrace,
            libc::SYS_process_vm_readv,
            libc::SYS_process_vm_writev,
            libc::SYS_process_madvise,
            libc::SYS_pidfd_getfd,
            libc::SYS_kcmp,
            libc::SYS_clone,
            libc::SYS_clone3,
            libc::SYS_fork,
            libc::SYS_vfork,
            libc::SYS_execve,
            libc::SYS_execveat,
            libc::SYS_unshare,
            libc::SYS_setns,
            libc::SYS_mount,
            libc::SYS_umount2,
            libc::SYS_pivot_root,
            libc::SYS_chroot,
        ];

        for syscall in FORBIDDEN_RENDERER_SYSCALLS {
            if self.allows(*syscall) {
                return Err(SeccompError::ForbiddenRendererSyscall(*syscall));
            }
        }
        Ok(())
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
    PolicyCommitmentMismatch,
    InvalidAssignmentId,
    ForbiddenRendererSyscall(i64),
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
            Self::PolicyCommitmentMismatch => f.write_str("seccomp policy does not match the renderer profile commitment"),
            Self::InvalidAssignmentId => f.write_str("renderer process assignment id must be non-zero"),
            Self::ForbiddenRendererSyscall(syscall) => write!(f, "renderer seccomp policy forbids syscall {syscall}"),
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
    const BPF_JGE: u16 = 0x30;
    const BPF_K: u16 = 0x00;
    const BPF_RET: u16 = 0x06;

    const SECCOMP_SET_MODE_FILTER: libc::c_uint = 1;
    const SECCOMP_FILTER_FLAG_TSYNC: libc::c_uint = 1 << 0;
    const SECCOMP_FILTER_FLAG_TSYNC_ESRCH: libc::c_uint = 1 << 4;
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

    fn jump_ge(k: u32, jt: u8, jf: u8) -> SockFilter {
        SockFilter { code: BPF_JMP | BPF_JGE | BPF_K, jt, jf, k }
    }

    fn compile_filter(policy: &SeccompSyscallPolicyV1) -> Result<Vec<SockFilter>, SeccompError> {
        if SeccompArchitecture::current() != Some(policy.architecture) {
            return Err(SeccompError::ArchitectureMismatch);
        }

        // Architecture check plus an x32-ABI guard where required, two
        // instructions per allowlisted syscall (match + ALLOW), followed by
        // the bounded default-deny action. Keeping the match/ALLOW pair
        // adjacent avoids jump-offset overflow while making the control flow
        // mechanically auditable.
        let abi_guard_instructions = if policy.architecture == SeccompArchitecture::X86_64 {
            2usize
        } else {
            0usize
        };
        let instruction_count = 5usize
            .saturating_add(abi_guard_instructions)
            .saturating_add(policy.allowed_syscalls.len().saturating_mul(2));
        if instruction_count > 4096 || instruction_count > u16::MAX as usize {
            return Err(SeccompError::FilterTooLarge);
        }

        let mut filter = Vec::with_capacity(instruction_count);
        filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, SECCOMP_DATA_ARCH_OFFSET));
        filter.push(jump_eq(policy.architecture as u32, 1, 0));
        filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_KILL_PROCESS));
        filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, SECCOMP_DATA_NR_OFFSET));

        if policy.architecture == SeccompArchitecture::X86_64 {
            const X32_SYSCALL_BIT: u32 = 0x4000_0000;
            // x32 uses the same audit architecture value but sets bit 30 in
            // the syscall number. Kill it explicitly before the allowlist.
            filter.push(jump_ge(X32_SYSCALL_BIT, 0, 1));
            filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_KILL_PROCESS));
        }

        for syscall in &policy.allowed_syscalls {
            // Match -> next instruction is ALLOW; mismatch skips that ALLOW
            // and continues with the next syscall comparison.
            filter.push(jump_eq(*syscall as u32, 0, 1));
            filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_ALLOW));
        }

        filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | libc::EPERM as u32));

        Ok(filter)
    }

    fn seccomp_evidence_digest(
        policy: &SeccompSyscallPolicyV1,
        filter: &[SockFilter],
    ) -> [u8; 32] {
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"PRISM-SECCOMP-SYSCALL-EVIDENCE-V3");
        hasher.update(&policy.digest());
        hasher.update(&(filter.len() as u32).to_le_bytes());
        for instruction in filter {
            hasher.update(&instruction.code.to_le_bytes());
            hasher.update(&[instruction.jt, instruction.jf]);
            hasher.update(&instruction.k.to_le_bytes());
        }
        // This commits the renderer-policy validation contract separately
        // from the syscall list and compiled BPF. Future changes to the
        // forbidden renderer syscall set must therefore bump this version.
        hasher.update(b"PRISM-SECCOMP-RENDERER-POLICY-VALIDATION-V1");
        hasher.update(b"PRISM-SECCOMP-NO-NEW-PRIVS-REQUIRED-V1");
        hasher.update(&SECCOMP_RET_ERRNO.to_le_bytes());
        hasher.update(&(libc::EPERM as u32).to_le_bytes());
        hasher.update(&SECCOMP_RET_KILL_PROCESS.to_le_bytes());
        hasher.update(&SECCOMP_RET_ALLOW.to_le_bytes());
        hasher.update(&(SECCOMP_FILTER_FLAG_TSYNC | SECCOMP_FILTER_FLAG_TSYNC_ESRCH).to_le_bytes());
        *hasher.finalize().as_bytes()
    }

    /// Install an explicit architecture-qualified syscall allowlist into the
    /// current process. This call is intended for the renderer bootstrap after
    /// process identity is established and before renderer capability IPC.
    pub fn install(
        assignment_id: RendererProcessAssignmentId,
        profile: SandboxProfileV1,
        policy: &SeccompSyscallPolicyV1,
    ) -> Result<SandboxEnforcementReceipt, SeccompError> {
        // The renderer profile must commit to the exact syscall policy that
        // this adapter is about to install. Reject mismatches before any
        // irreversible state transition and before generating installation
        // identity material that may itself become unavailable after filtering.
        if assignment_id.0 == 0 {
            return Err(SeccompError::InvalidAssignmentId);
        }

        if profile.syscall_policy_digest() == [0u8; 32]
            || policy.digest() != profile.syscall_policy_digest()
        {
            return Err(SeccompError::PolicyCommitmentMismatch);
        }

        policy.validate_renderer_policy()?;

        // Generate the installation identity before irreversible enforcement.
        // Once seccomp is installed, a policy may legitimately deny getrandom(2)
        // and other runtime helpers. A post-install RNG failure must never leave
        // the caller sandboxed while the adapter reports installation failure.
        let mut bytes = [0u8; 16];
        getrandom::fill(&mut bytes).map_err(|_| SeccompError::IdentityGenerationFailed)?;
        let installation_id = SandboxInstallationId::new(u128::from_be_bytes(bytes))
            .map_err(|_| SeccompError::IdentityGenerationFailed)?;

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

        // A renderer may be multithreaded. prctl(PR_SET_SECCOMP) can only
        // attach to the calling thread; use the seccomp() API with TSYNC so
        // the broker never records process-level syscall evidence for a
        // single-thread-only filter. TSYNC_ESRCH normalizes synchronization
        // failure to errno instead of exposing a kernel TID as the return value.
        let rc = unsafe {
            libc::syscall(
                libc::SYS_seccomp,
                SECCOMP_SET_MODE_FILTER,
                SECCOMP_FILTER_FLAG_TSYNC | SECCOMP_FILTER_FLAG_TSYNC_ESRCH,
                &program as *const SockFprog,
            )
        };
        if rc < 0 {
            return Err(SeccompError::InstallationFailed(
                std::io::Error::last_os_error().raw_os_error().unwrap_or(libc::EPERM),
            ));
        }
        if rc > 0 {
            // TSYNC_ESRCH asks the kernel to report synchronization failure as
            // ESRCH rather than exposing a kernel TID. Treat any positive
            // result defensively as a synchronization failure too; never
            // interpret a thread ID as successful enforcement.
            return Err(SeccompError::InstallationFailed(libc::ESRCH));
        }

        SandboxEnforcementReceipt::from_adapter(
            assignment_id,
            installation_id,
            SandboxAdapterKind::LinuxSeccompSyscallV1,
            profile.policy_digest(),
            seccomp_evidence_digest(policy, &filter),
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
            assert_eq!(policy.allowed_syscalls(), &[3, 7, 9]);
            assert!(SeccompSyscallPolicyV1::new(arch, vec![3, 3]).is_err());
        }

        #[test]
        fn each_allowlisted_syscall_has_a_reachable_allow_action() {
            let arch = SeccompArchitecture::current().unwrap();
            let policy = SeccompSyscallPolicyV1::new(arch, vec![libc::SYS_read, libc::SYS_write]).unwrap();
            let filter = compile_filter(&policy).unwrap();

            // x86-64: 0 load arch, 1 arch match, 2 kill on mismatch,
            // 3 load nr, 4 x32 guard, 5 kill x32, then JEQ -> ALLOW pairs.
            // Mismatch skips exactly one ALLOW and reaches the next comparison.
            if arch == SeccompArchitecture::X86_64 {
                assert_eq!(filter[4].code, BPF_JMP | BPF_JGE | BPF_K);
                assert_eq!(filter[4].k, 0x4000_0000);
                assert_eq!(filter[4].jt, 0);
                assert_eq!(filter[4].jf, 1);
                assert_eq!(filter[5].k, SECCOMP_RET_KILL_PROCESS);
                assert_eq!(filter[6].jt, 0);
                assert_eq!(filter[6].jf, 1);
                assert_eq!(filter[7].k, SECCOMP_RET_ALLOW);
                assert_eq!(filter[8].jt, 0);
                assert_eq!(filter[8].jf, 1);
                assert_eq!(filter[9].k, SECCOMP_RET_ALLOW);
                assert_eq!(filter[10].k, SECCOMP_RET_ERRNO | libc::EPERM as u32);
                assert_eq!(filter.len(), 11);
            }
        }

        #[test]
        fn oversized_syscall_number_is_rejected() {
            let policy = SeccompSyscallPolicyV1::new(
                SeccompArchitecture::X86_64,
                vec![i32::MAX as i64 + 1],
            );
            assert!(matches!(policy, Err(SeccompError::InvalidPolicy)));
        }

        #[test]
        fn oversized_policy_is_rejected() {
            let syscalls = (0..257).map(|n| n as i64).collect();
            let policy = SeccompSyscallPolicyV1::new(
                SeccompArchitecture::X86_64,
                syscalls,
            );
            assert!(matches!(policy, Err(SeccompError::InvalidPolicy)));
        }

        #[test]
        fn x32_tagged_syscalls_are_rejected_from_x86_policy() {
            let policy = SeccompSyscallPolicyV1::new(
                SeccompArchitecture::X86_64,
                vec![0x4000_0000],
            );
            assert!(matches!(policy, Err(SeccompError::InvalidPolicy)));
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
        fn uncommitted_profile_rejects_install_before_enforcement() {
            let arch = SeccompArchitecture::current().unwrap();
            let policy = SeccompSyscallPolicyV1::new(arch, vec![libc::SYS_getpid]).unwrap();
            let profile = SandboxProfileV1::renderer_default();
            assert!(matches!(
                install(
                    RendererProcessAssignmentId::new(1).unwrap(),
                    profile,
                    &policy,
                ),
                Err(SeccompError::PolicyCommitmentMismatch)
            ));
        }

        #[test]
        fn renderer_policy_rejects_process_tracing_primitives() {
            let arch = SeccompArchitecture::current().unwrap();
            let policy = SeccompSyscallPolicyV1::new(
                arch,
                vec![libc::SYS_getpid, libc::SYS_ptrace],
            )
            .unwrap();
            assert!(matches!(
                policy.validate_renderer_policy(),
                Err(SeccompError::ForbiddenRendererSyscall(syscall))
                    if syscall == libc::SYS_ptrace
            ));
        }

        #[test]
        fn zero_assignment_rejects_install_before_enforcement() {
            let arch = SeccompArchitecture::current().unwrap();
            let policy = SeccompSyscallPolicyV1::new(
                arch,
                vec![libc::SYS_getpid],
            )
            .unwrap();
            let profile = SandboxProfileV1::renderer_default()
                .with_syscall_policy_digest(policy.digest())
                .unwrap();

            assert!(matches!(
                install(
                    RendererProcessAssignmentId(0),
                    profile,
                    &policy,
                ),
                Err(SeccompError::InvalidAssignmentId)
            ));
        }

        #[test]
        fn forbidden_renderer_policy_rejects_install_before_enforcement() {
            let arch = SeccompArchitecture::current().unwrap();
            let policy = SeccompSyscallPolicyV1::new(
                arch,
                vec![libc::SYS_getpid, libc::SYS_ptrace],
            )
            .unwrap();
            let profile = SandboxProfileV1::renderer_default()
                .with_syscall_policy_digest(policy.digest())
                .unwrap();

            assert!(matches!(
                install(
                    RendererProcessAssignmentId::new(1).unwrap(),
                    profile,
                    &policy,
                ),
                Err(SeccompError::ForbiddenRendererSyscall(syscall))
                    if syscall == libc::SYS_ptrace
            ));
        }

        #[test]
        fn renderer_policy_rejects_process_creation_and_namespace_mutation() {
            let arch = SeccompArchitecture::current().unwrap();

            for syscall in [
                libc::SYS_clone,
                libc::SYS_clone3,
                libc::SYS_fork,
                libc::SYS_vfork,
                libc::SYS_execve,
                libc::SYS_execveat,
                libc::SYS_unshare,
                libc::SYS_setns,
            ] {
                let policy = SeccompSyscallPolicyV1::new(
                    arch,
                    vec![libc::SYS_getpid, syscall],
                )
                .unwrap();
                assert!(matches!(
                    policy.validate_renderer_policy(),
                    Err(SeccompError::ForbiddenRendererSyscall(rejected))
                        if rejected == syscall
                ));
            }
        }

        #[test]
        fn profile_policy_digest_is_part_of_profile_commitment() {
            let arch = SeccompArchitecture::current().unwrap();
            let policy = SeccompSyscallPolicyV1::new(arch, vec![libc::SYS_getpid]).unwrap();
            let committed = SandboxProfileV1::renderer_default()
                .with_syscall_policy_digest(policy.digest())
                .unwrap();
            let other = SeccompSyscallPolicyV1::new(arch, vec![libc::SYS_getppid]).unwrap();
            assert_ne!(committed.syscall_policy_digest(), other.digest());
            assert_ne!(committed.policy_digest(), SandboxProfileV1::renderer_default().policy_digest());
        }

        #[test]
        fn evidence_digest_is_distinct_from_policy_digest() {
            let arch = SeccompArchitecture::current().unwrap();
            let policy = SeccompSyscallPolicyV1::new(arch, vec![libc::SYS_getpid]).unwrap();
            let filter = compile_filter(&policy).unwrap();
            assert_ne!(seccomp_evidence_digest(&policy, &filter), policy.digest());
        }

        #[test]
        fn digest_is_domain_separated_from_the_legacy_unversioned_encoding() {
            let arch = SeccompArchitecture::current().unwrap();
            let policy = SeccompSyscallPolicyV1::new(arch, vec![1, 2]).unwrap();

            let mut legacy = Vec::with_capacity(4 + policy.allowed_syscalls.len() * 4);
            legacy.extend_from_slice(&(policy.architecture as u32).to_le_bytes());
            for syscall in &policy.allowed_syscalls {
                legacy.extend_from_slice(&(*syscall as u32).to_le_bytes());
            }

            assert_ne!(policy.digest(), *blake3::hash(&legacy).as_bytes());
        }

        #[test]
        fn digest_changes_with_policy() {
            let arch = SeccompArchitecture::current().unwrap();
            let a = SeccompSyscallPolicyV1::new(arch, vec![1, 2]).unwrap();
            let b = SeccompSyscallPolicyV1::new(arch, vec![1, 3]).unwrap();
            assert_ne!(a.digest(), b.digest());
        }
    }

    #[cfg(test)]
    #[test]
    fn linux_install_export_is_present() {
        #[cfg(target_os = "linux")]
        {
            let _ = install;
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
