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
        for syscall in FORBIDDEN_RENDERER_SYSCALLS {
            if self.allows(*syscall) {
                return Err(SeccompError::ForbiddenRendererSyscall(*syscall));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SeccompArgPredicateOpV1 {
    MaskedEqual,
    MaskedNotEqual,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SeccompArgPredicateV1 {
    arg_index: u8,
    mask: u64,
    value: u64,
    op: SeccompArgPredicateOpV1,
}

impl SeccompArgPredicateV1 {
    pub fn new(arg_index: u8, mask: u64, value: u64) -> Result<Self, SeccompError> {
        Self::new_with_op(arg_index, mask, value, SeccompArgPredicateOpV1::MaskedEqual)
    }

    pub fn new_with_op(
        arg_index: u8,
        mask: u64,
        value: u64,
        op: SeccompArgPredicateOpV1,
    ) -> Result<Self, SeccompError> {
        const MAX_ARGS: u8 = 6;
        if arg_index >= MAX_ARGS || mask == 0 || value & !mask != 0 {
            return Err(SeccompError::InvalidPolicy);
        }
        Ok(Self { arg_index, mask, value, op })
    }

    pub const fn arg_index(&self) -> u8 { self.arg_index }
    pub const fn mask(&self) -> u64 { self.mask }
    pub const fn value(&self) -> u64 { self.value }
    pub const fn op(&self) -> SeccompArgPredicateOpV1 { self.op }

    pub const fn matches(&self, argument: u64) -> bool {
        match self.op {
            SeccompArgPredicateOpV1::MaskedEqual => argument & self.mask == self.value,
            SeccompArgPredicateOpV1::MaskedNotEqual => argument & self.mask != self.value,
        }
    }
}

/// One V2 syscall rule. An empty predicate list means the syscall itself is
/// allowed; once predicates are present, every predicate must match.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SeccompSyscallRuleV2 {
    syscall: i64,
    predicates: Vec<SeccompArgPredicateV1>,
}

impl SeccompSyscallRuleV2 {
    pub fn new(
        syscall: i64,
        mut predicates: Vec<SeccompArgPredicateV1>,
    ) -> Result<Self, SeccompError> {
        if syscall < 0 || syscall > i32::MAX as i64 {
            return Err(SeccompError::InvalidPolicy);
        }
        const MAX_PREDICATES: usize = 4;
        if predicates.len() > MAX_PREDICATES {
            return Err(SeccompError::InvalidPolicy);
        }
        predicates.sort_unstable_by_key(|predicate| {
            (predicate.arg_index, predicate.mask, predicate.value)
        });
        if predicates.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(SeccompError::DuplicateArgumentPredicate);
        }
        Ok(Self { syscall, predicates })
    }

    pub const fn syscall(&self) -> i64 { self.syscall }

    pub fn predicates(&self) -> &[SeccompArgPredicateV1] {
        &self.predicates
    }

    pub fn matches(&self, syscall: i64, arguments: &[u64; 6]) -> bool {
        self.syscall == syscall
            && self.predicates.iter().all(|predicate| {
                predicate.matches(arguments[predicate.arg_index as usize])
            })
    }
}

/// Parameter-aware seccomp policy. V1 remains the syscall-number-only format;
/// V2 adds explicit argument predicates without silently widening V1.
#[cfg(target_os = "linux")]
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

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SeccompSyscallPolicyV2 {
    architecture: SeccompArchitecture,
    rules: Vec<SeccompSyscallRuleV2>,
}

impl SeccompSyscallPolicyV2 {
    pub fn new(
        architecture: SeccompArchitecture,
        mut rules: Vec<SeccompSyscallRuleV2>,
    ) -> Result<Self, SeccompError> {
        const MAX_RULES: usize = 64;
        const X32_SYSCALL_BIT: i64 = 0x4000_0000;
        if rules.is_empty() || rules.len() > MAX_RULES {
            return Err(SeccompError::InvalidPolicy);
        }
        if architecture == SeccompArchitecture::X86_64
            && rules.iter().any(|rule| (rule.syscall & X32_SYSCALL_BIT) != 0)
        {
            return Err(SeccompError::InvalidPolicy);
        }
        rules.sort_unstable_by_key(|rule| rule.syscall);
        if rules.windows(2).any(|pair| pair[0].syscall == pair[1].syscall) {
            return Err(SeccompError::DuplicateSyscall);
        }
        Ok(Self { architecture, rules })
    }

    pub const fn architecture(&self) -> SeccompArchitecture {
        self.architecture
    }

    pub fn rules(&self) -> &[SeccompSyscallRuleV2] {
        &self.rules
    }

    pub fn allows(&self, syscall: i64, arguments: &[u64; 6]) -> bool {
        self.rules.iter().any(|rule| rule.matches(syscall, arguments))
    }

    pub fn digest(&self) -> [u8; 32] {
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"PRISM-SECCOMP-SYSCALL-POLICY-V2");
        if cfg!(target_endian = "little") {
            hasher.update(b"PRISM-SECCOMP-ENDIAN-LITTLE-V1");
        } else {
            hasher.update(b"PRISM-SECCOMP-ENDIAN-BIG-V1");
        }
        hasher.update(&(self.architecture as u32).to_le_bytes());
        hasher.update(&(self.rules.len() as u32).to_le_bytes());
        for rule in &self.rules {
            hasher.update(&(rule.syscall as u32).to_le_bytes());
            hasher.update(&(rule.predicates.len() as u32).to_le_bytes());
            for predicate in &rule.predicates {
                hasher.update(&(predicate.arg_index as u32).to_le_bytes());
                hasher.update(&predicate.mask.to_le_bytes());
                hasher.update(&predicate.value.to_le_bytes());
                hasher.update(&[match predicate.op {
                    SeccompArgPredicateOpV1::MaskedEqual => 0,
                    SeccompArgPredicateOpV1::MaskedNotEqual => 1,
                }]);
            }
        }
        *hasher.finalize().as_bytes()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeccompError {
    UnsupportedPlatform,
    ArchitectureMismatch,
    InvalidPolicy,
    DuplicateSyscall,
    DuplicateArgumentPredicate,
    FilterTooLarge,
    InstallationFailed(i32),
    IdentityGenerationFailed,
    PolicyCommitmentMismatch,
    InvalidAssignmentId,
    UnsupportedEndianness,
    ForbiddenRendererSyscall(i64),
}

impl core::fmt::Display for SeccompError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnsupportedPlatform => f.write_str("seccomp unsupported on this platform"),
            Self::ArchitectureMismatch => f.write_str("seccomp policy architecture does not match the running architecture"),
            Self::InvalidPolicy => f.write_str("invalid seccomp syscall policy"),
            Self::DuplicateSyscall => f.write_str("seccomp syscall policy contains a duplicate"),
            Self::DuplicateArgumentPredicate => f.write_str("seccomp argument predicate contains a duplicate"),
            Self::FilterTooLarge => f.write_str("seccomp BPF filter exceeds the bounded instruction budget"),
            Self::InstallationFailed(errno) => write!(f, "seccomp installation failed: errno {errno}"),
            Self::IdentityGenerationFailed => f.write_str("seccomp installation identity generation failed"),
            Self::PolicyCommitmentMismatch => f.write_str("seccomp policy does not match the renderer profile commitment"),
            Self::InvalidAssignmentId => f.write_str("renderer process assignment id must be non-zero"),
            Self::UnsupportedEndianness => f.write_str("parameter-aware seccomp policy requires little-endian execution"),
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
    const BPF_ALU: u16 = 0x04;
    const BPF_AND: u16 = 0x50;

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

    fn compile_filter_v2(policy: &SeccompSyscallPolicyV2) -> Result<Vec<SockFilter>, SeccompError> {
        #[cfg(target_endian = "big")]
        return Err(SeccompError::UnsupportedEndianness);

        if SeccompArchitecture::current() != Some(policy.architecture) {
            return Err(SeccompError::ArchitectureMismatch);
        }

        const ARG_BASE: u32 = 16;
        let instruction_count = 5usize
            .saturating_add(if policy.architecture == SeccompArchitecture::X86_64 { 2 } else { 0 })
            .saturating_add(
                policy.rules.iter().map(|rule| {
                    2usize
                        + rule.predicates.iter().map(|predicate| {
                            usize::from(predicate.mask as u32 != 0)
                                .saturating_mul(4)
                                .saturating_add(usize::from((predicate.mask >> 32) as u32 != 0).saturating_mul(4))
                        }).sum::<usize>()
                }).sum::<usize>(),
            );
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
            filter.push(jump_ge(X32_SYSCALL_BIT, 0, 1));
            filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_KILL_PROCESS));
        }

        for rule in &policy.rules {
            let predicate_instructions = rule.predicates.iter().map(|predicate| {
                let low = predicate.mask as u32;
                let high = (predicate.mask >> 32) as u32;
                usize::from(low != 0) * 4 + usize::from(high != 0) * 4
            }).sum::<usize>();
            let body_len = predicate_instructions
                .checked_add(1)
                .ok_or(SeccompError::FilterTooLarge)?;
            let body_jump = u8::try_from(body_len).map_err(|_| SeccompError::FilterTooLarge)?;
            filter.push(jump_eq(rule.syscall as u32, 0, body_jump));

            for predicate in &rule.predicates {
                let base = ARG_BASE + u32::from(predicate.arg_index) * 8;
                let low_mask = predicate.mask as u32;
                let low_value = predicate.value as u32;
                let high_mask = (predicate.mask >> 32) as u32;
                let high_value = (predicate.value >> 32) as u32;

                let reject_on_equal = predicate.op == SeccompArgPredicateOpV1::MaskedNotEqual;
                let predicate_jump = |filter: &mut Vec<SockFilter>, value: u32| {
                    let (jt, jf) = if reject_on_equal { (0, 1) } else { (1, 0) };
                    filter.push(jump_eq(value, jt, jf));
                    filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | libc::EPERM as u32));
                };
                if low_mask != 0 {
                    filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, base));
                    filter.push(stmt(BPF_ALU | BPF_AND | BPF_K, low_mask));
                    predicate_jump(&mut filter, low_value);
                }
                if high_mask != 0 {
                    filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, base + 4));
                    filter.push(stmt(BPF_ALU | BPF_AND | BPF_K, high_mask));
                    predicate_jump(&mut filter, high_value);
                }
            }

            filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_ALLOW));
        }

        filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | libc::EPERM as u32));
        Ok(filter)
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

    fn seccomp_evidence_digest_v2(
        policy: &SeccompSyscallPolicyV2,
        filter: &[SockFilter],
    ) -> [u8; 32] {
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"PRISM-SECCOMP-SYSCALL-EVIDENCE-V4");
        hasher.update(&policy.digest());
        hasher.update(&(filter.len() as u32).to_le_bytes());
        for instruction in filter {
            hasher.update(&instruction.code.to_le_bytes());
            hasher.update(&[instruction.jt, instruction.jf]);
            hasher.update(&instruction.k.to_le_bytes());
        }
        hasher.update(b"PRISM-SECCOMP-RENDERER-POLICY-VALIDATION-V1");
        hasher.update(b"PRISM-SECCOMP-NO-NEW-PRIVS-REQUIRED-V1");
        hasher.update(&SECCOMP_RET_ERRNO.to_le_bytes());
        hasher.update(&(libc::EPERM as u32).to_le_bytes());
        hasher.update(&SECCOMP_RET_KILL_PROCESS.to_le_bytes());
        hasher.update(&SECCOMP_RET_ALLOW.to_le_bytes());
        hasher.update(&(SECCOMP_FILTER_FLAG_TSYNC | SECCOMP_FILTER_FLAG_TSYNC_ESRCH).to_le_bytes());
        *hasher.finalize().as_bytes()
    }

    fn validate_v2_renderer_policy(policy: &SeccompSyscallPolicyV2) -> Result<(), SeccompError> {
        if let Some(rule) = policy.rules.iter().find(|rule| {
            FORBIDDEN_RENDERER_SYSCALLS.contains(&rule.syscall)
        }) {
            return Err(SeccompError::ForbiddenRendererSyscall(rule.syscall));
        }
        Ok(())
    }

    /// Install a parameter-aware V2 policy. V1 remains the stable
    /// syscall-number-only installation path; V2 must commit its complete
    /// predicate-bearing policy to the renderer profile before enforcement.
    pub fn install_v2(
        assignment_id: RendererProcessAssignmentId,
        profile: SandboxProfileV1,
        policy: &SeccompSyscallPolicyV2,
    ) -> Result<SandboxEnforcementReceipt, SeccompError> {
        if assignment_id.0 == 0 {
            return Err(SeccompError::InvalidAssignmentId);
        }
        if profile.syscall_policy_digest() == [0u8; 32]
            || policy.digest() != profile.syscall_policy_digest()
        {
            return Err(SeccompError::PolicyCommitmentMismatch);
        }
        validate_v2_renderer_policy(policy)?;

        let mut bytes = [0u8; 16];
        getrandom::fill(&mut bytes).map_err(|_| SeccompError::IdentityGenerationFailed)?;
        let installation_id = SandboxInstallationId::new(u128::from_be_bytes(bytes))
            .map_err(|_| SeccompError::IdentityGenerationFailed)?;

        let mut filter = compile_filter_v2(policy)?;

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
            return Err(SeccompError::InstallationFailed(libc::ESRCH));
        }

        SandboxEnforcementReceipt::from_adapter(
            assignment_id,
            installation_id,
            SandboxAdapterKind::LinuxSeccompSyscallV2,
            profile.policy_digest(),
            seccomp_evidence_digest_v2(policy, &filter),
            SandboxEnforcementLayer::Syscall,
        ).map_err(|_| SeccompError::InvalidPolicy)
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
        fn v2_masked_not_equal_predicate_supports_forbidden_bit_combinations() {
            let predicate = SeccompArgPredicateV1::new_with_op(
                2,
                (libc::PROT_WRITE | libc::PROT_EXEC) as u64,
                (libc::PROT_WRITE | libc::PROT_EXEC) as u64,
                SeccompArgPredicateOpV1::MaskedNotEqual,
            ).unwrap();

            assert!(!predicate.matches((libc::PROT_WRITE | libc::PROT_EXEC) as u64));
            assert!(predicate.matches(libc::PROT_READ as u64));
            assert!(predicate.matches(libc::PROT_EXEC as u64));
        }

        #[test]
        fn v2_predicates_are_canonical_and_argument_bound() {
            let predicate = SeccompArgPredicateV1::new(0, 0x0000_ffff, 0x0000_1234).unwrap();
            assert!(predicate.matches(0x1234));
            assert!(predicate.matches(0xabcd_1234));
            assert!(!predicate.matches(0x1235));

            let rule = SeccompSyscallRuleV2::new(
                libc::SYS_prctl,
                vec![
                    predicate,
                    SeccompArgPredicateV1::new(1, u64::MAX, 0).unwrap(),
                ],
            )
            .unwrap();
            assert_eq!(rule.predicates().len(), 2);

            let policy = SeccompSyscallPolicyV2::new(
                SeccompArchitecture::current().unwrap(),
                vec![rule],
            )
            .unwrap();
            assert!(policy.allows(libc::SYS_prctl, &[0x1234, 0, 0, 0, 0, 0]));
            assert!(!policy.allows(libc::SYS_prctl, &[0x1235, 0, 0, 0, 0, 0]));
        }

        #[test]
        fn v2_rejects_duplicate_rules_and_predicates() {
            let arch = SeccompArchitecture::current().unwrap();
            let predicate = SeccompArgPredicateV1::new(0, u64::MAX, 1).unwrap();
            let rule = SeccompSyscallRuleV2::new(libc::SYS_prctl, vec![predicate, predicate]);
            assert!(matches!(rule, Err(SeccompError::DuplicateArgumentPredicate)));

            let a = SeccompSyscallRuleV2::new(libc::SYS_prctl, vec![SeccompArgPredicateV1::new(0, 1, 1).unwrap()]).unwrap();
            let b = SeccompSyscallRuleV2::new(libc::SYS_prctl, vec![SeccompArgPredicateV1::new(1, 1, 1).unwrap()]).unwrap();
            assert!(matches!(
                SeccompSyscallPolicyV2::new(arch, vec![a, b]),
                Err(SeccompError::DuplicateSyscall)
            ));
        }

        #[test]
        fn v2_digest_is_deterministic_and_domain_separated() {
            let arch = SeccompArchitecture::current().unwrap();
            let a = SeccompSyscallRuleV2::new(
                libc::SYS_prctl,
                vec![SeccompArgPredicateV1::new(0, 0xff, 1).unwrap()],
            ).unwrap();
            let b = SeccompSyscallRuleV2::new(
                libc::SYS_exit_group,
                Vec::new(),
            ).unwrap();
            let first = SeccompSyscallPolicyV2::new(arch, vec![b.clone(), a.clone()]).unwrap();
            let second = SeccompSyscallPolicyV2::new(arch, vec![a, b]).unwrap();
            assert_eq!(first.digest(), second.digest());

            let v1 = SeccompSyscallPolicyV1::new(arch, vec![libc::SYS_prctl, libc::SYS_exit_group]).unwrap();
            assert_ne!(first.digest(), v1.digest());
        }

        #[test]
        fn v2_install_requires_exact_profile_commitment() {
            let arch = SeccompArchitecture::current().unwrap();
            let rule = SeccompSyscallRuleV2::new(
                libc::SYS_prctl,
                vec![SeccompArgPredicateV1::new(0, u64::MAX, libc::PR_GET_NO_NEW_PRIVS as u64).unwrap()],
            ).unwrap();
            let policy = SeccompSyscallPolicyV2::new(arch, vec![rule]).unwrap();
            let profile = SandboxProfileV1::renderer_default();
            assert!(matches!(
                install_v2(RendererProcessAssignmentId::new(5).unwrap(), profile, &policy),
                Err(SeccompError::PolicyCommitmentMismatch)
            ));
        }

        #[test]
        fn v2_install_rejects_forbidden_renderer_syscalls_before_enforcement() {
            let arch = SeccompArchitecture::current().unwrap();
            let rule = SeccompSyscallRuleV2::new(
                libc::SYS_ptrace,
                Vec::new(),
            ).unwrap();
            let policy = SeccompSyscallPolicyV2::new(arch, vec![rule]).unwrap();
            let profile = SandboxProfileV1::renderer_default()
                .with_syscall_policy_digest(policy.digest())
                .unwrap();

            assert!(matches!(
                install_v2(RendererProcessAssignmentId::new(6).unwrap(), profile, &policy),
                Err(SeccompError::ForbiddenRendererSyscall(syscall))
                    if syscall == libc::SYS_ptrace
            ));
        }

        #[test]
        fn v2_evidence_digest_is_distinct_from_v2_policy_digest() {
            let arch = SeccompArchitecture::current().unwrap();
            let rule = SeccompSyscallRuleV2::new(
                libc::SYS_prctl,
                vec![SeccompArgPredicateV1::new(0, u64::MAX, libc::PR_GET_NO_NEW_PRIVS as u64).unwrap()],
            ).unwrap();
            let policy = SeccompSyscallPolicyV2::new(arch, vec![rule]).unwrap();
            let filter = compile_filter_v2(&policy).unwrap();
            assert_ne!(seccomp_evidence_digest_v2(&policy, &filter), policy.digest());
        }

        #[test]
        fn v2_policy_digest_commits_endian_domain() {
            let arch = SeccompArchitecture::current().unwrap();
            let rule = SeccompSyscallRuleV2::new(
                libc::SYS_prctl,
                vec![SeccompArgPredicateV1::new(1, 0xff00_0000_0000_0000, 0x1200_0000_0000_0000).unwrap()],
            ).unwrap();
            let policy = SeccompSyscallPolicyV2::new(arch, vec![rule]).unwrap();

            let mut legacy = blake3::Hasher::new();
            legacy.update(b"PRISM-SECCOMP-SYSCALL-POLICY-V2");
            legacy.update(&(arch as u32).to_le_bytes());
            legacy.update(&(policy.rules().len() as u32).to_le_bytes());
            for rule in policy.rules() {
                legacy.update(&(rule.syscall() as u32).to_le_bytes());
                legacy.update(&(rule.predicates().len() as u32).to_le_bytes());
                for predicate in rule.predicates() {
                    legacy.update(&(predicate.arg_index() as u32).to_le_bytes());
                    legacy.update(&predicate.mask().to_le_bytes());
                    legacy.update(&predicate.value().to_le_bytes());
                }
            }

            assert_ne!(policy.digest(), *legacy.finalize().as_bytes());
        }

        #[test]
        fn v2_compiler_rejects_wrong_architecture() {
            let current = SeccompArchitecture::current().unwrap();
            let other = match current {
                SeccompArchitecture::X86_64 => SeccompArchitecture::Aarch64,
                _ => SeccompArchitecture::X86_64,
            };
            let rule = SeccompSyscallRuleV2::new(
                libc::SYS_prctl,
                vec![SeccompArgPredicateV1::new(0, u64::MAX, 0).unwrap()],
            ).unwrap();
            let policy = SeccompSyscallPolicyV2::new(other, vec![rule]).unwrap();
            assert!(matches!(
                compile_filter_v2(&policy),
                Err(SeccompError::ArchitectureMismatch)
            ));
        }

        #[test]
        fn v2_compiler_stays_within_bpf_budget_at_policy_limit() {
            let arch = SeccompArchitecture::current().unwrap();
            let mut rules = Vec::new();
            for syscall in 0..64 {
                rules.push(SeccompSyscallRuleV2::new(
                    10_000 + syscall,
                    vec![
                        SeccompArgPredicateV1::new(0, u64::MAX, 0).unwrap(),
                        SeccompArgPredicateV1::new(1, u64::MAX, 1).unwrap(),
                        SeccompArgPredicateV1::new(2, u64::MAX, 2).unwrap(),
                        SeccompArgPredicateV1::new(3, u64::MAX, 3).unwrap(),
                    ],
                ).unwrap());
            }
            let policy = SeccompSyscallPolicyV2::new(arch, rules).unwrap();
            let filter = compile_filter_v2(&policy).unwrap();
            assert!(filter.len() <= 4096);
        }

        #[test]
        fn v2_compiler_jump_offsets_skip_only_the_current_rule() {
            let arch = SeccompArchitecture::current().unwrap();
            let single = SeccompSyscallRuleV2::new(
                libc::SYS_prctl,
                vec![SeccompArgPredicateV1::new(0, u32::MAX as u64, 7).unwrap()],
            ).unwrap();
            let unfiltered = SeccompSyscallRuleV2::new(libc::SYS_getpid, Vec::new()).unwrap();
            let policy = SeccompSyscallPolicyV2::new(arch, vec![single, unfiltered]).unwrap();
            let filter = compile_filter_v2(&policy).unwrap();

            // Find the first syscall dispatch comparison and assert that a
            // syscall mismatch skips exactly the predicate body plus its ALLOW,
            // landing on the next syscall comparison.
            let prctl_index = filter.iter().position(|instruction| {
                instruction.code == BPF_JMP | BPF_JEQ | BPF_K
                    && instruction.k == libc::SYS_prctl as u32
            }).unwrap();
            let predicate_body = filter[prctl_index + 1..].iter().position(|instruction| {
                instruction.code == BPF_JMP | BPF_JEQ | BPF_K
                    && instruction.k == libc::SYS_getpid as u32
            }).unwrap();
            assert_eq!(filter[prctl_index].jf as usize, predicate_body + 1);

            let getpid_index = prctl_index + 1 + predicate_body;
            assert_eq!(
                filter[getpid_index].k,
                libc::SYS_getpid as u32
            );
        }

        #[test]
        fn v2_compiler_binds_argument_words_before_allow() {
            let arch = SeccompArchitecture::current().unwrap();
            let rule = SeccompSyscallRuleV2::new(
                libc::SYS_prctl,
                vec![SeccompArgPredicateV1::new(0, u64::MAX, libc::PR_GET_NO_NEW_PRIVS as u64).unwrap()],
            ).unwrap();
            let policy = SeccompSyscallPolicyV2::new(arch, vec![rule]).unwrap();
            let filter = compile_filter_v2(&policy).unwrap();

            assert!(filter.iter().any(|instruction| {
                instruction.code == BPF_LD | BPF_W | BPF_ABS && instruction.k == 16
            }));
            assert!(filter.iter().any(|instruction| {
                instruction.code == BPF_LD | BPF_W | BPF_ABS && instruction.k == 20
            }));
            assert!(filter.iter().any(|instruction| {
                instruction.code == BPF_ALU | BPF_AND | BPF_K && instruction.k == u32::MAX
            }));
            assert!(filter.iter().any(|instruction| {
                instruction.code == BPF_RET | BPF_K
                    && instruction.k == SECCOMP_RET_ERRNO | libc::EPERM as u32
            }));
            assert_eq!(filter.last().map(|instruction| instruction.k), Some(SECCOMP_RET_ERRNO | libc::EPERM as u32));
        }

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
pub use linux::{install, install_v2};

#[cfg(not(target_os = "linux"))]
pub fn install_v2(
    _assignment_id: RendererProcessAssignmentId,
    _profile: SandboxProfileV1,
    _policy: &SeccompSyscallPolicyV2,
) -> Result<SandboxEnforcementReceipt, SeccompError> {
    Err(SeccompError::UnsupportedPlatform)
}

#[cfg(not(target_os = "linux"))]
pub fn install(
    _assignment_id: RendererProcessAssignmentId,
    _profile: SandboxProfileV1,
    _policy: &SeccompSyscallPolicyV1,
) -> Result<SandboxEnforcementReceipt, SeccompError> {
    Err(SeccompError::UnsupportedPlatform)
}
