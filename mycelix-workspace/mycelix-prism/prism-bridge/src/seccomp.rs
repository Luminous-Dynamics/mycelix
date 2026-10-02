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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum SeccompArgPredicateOpV1 {
    MaskedEqual,
    MaskedNotEqual,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
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

/// One bounded V2 argument clause. All predicates in a clause must match.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct SeccompSyscallClauseV2 {
    predicates: Vec<SeccompArgPredicateV1>,
}

impl SeccompSyscallClauseV2 {
    pub fn new(mut predicates: Vec<SeccompArgPredicateV1>) -> Result<Self, SeccompError> {
        const MAX_PREDICATES: usize = 4;
        if predicates.len() > MAX_PREDICATES {
            return Err(SeccompError::InvalidPolicy);
        }
        predicates.sort_unstable_by_key(|predicate| {
            (predicate.arg_index, predicate.mask, predicate.value, predicate.op)
        });
        if predicates.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(SeccompError::DuplicateArgumentPredicate);
        }
        Ok(Self { predicates })
    }

    pub fn predicates(&self) -> &[SeccompArgPredicateV1] {
        &self.predicates
    }

    pub fn matches(&self, arguments: &[u64; 6]) -> bool {
        self.predicates.iter().all(|predicate| {
            predicate.matches(arguments[predicate.arg_index as usize])
        })
    }
}

/// One V2 syscall rule. A legacy single clause is equivalent to the original
/// conjunction. A bounded multi-clause rule is an explicit OR over clauses.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SeccompSyscallRuleV2 {
    syscall: i64,
    clauses: Vec<SeccompSyscallClauseV2>,
}

impl SeccompSyscallRuleV2 {
    pub fn new(
        syscall: i64,
        predicates: Vec<SeccompArgPredicateV1>,
    ) -> Result<Self, SeccompError> {
        Self::new_with_clauses(syscall, vec![SeccompSyscallClauseV2::new(predicates)?])
    }

    pub fn new_with_clauses(
        syscall: i64,
        mut clauses: Vec<SeccompSyscallClauseV2>,
    ) -> Result<Self, SeccompError> {
        const MAX_CLAUSES: usize = 4;
        if syscall < 0 || syscall > i32::MAX as i64 {
            return Err(SeccompError::InvalidPolicy);
        }
        if clauses.is_empty() || clauses.len() > MAX_CLAUSES {
            return Err(SeccompError::InvalidPolicy);
        }
        // An empty clause is an unconditional syscall allow. Permit it only
        // as the sole clause; otherwise it would make every alternative after
        // it unreachable and silently widen the syscall boundary.
        if clauses.len() > 1 && clauses.iter().any(|clause| clause.predicates.is_empty()) {
            return Err(SeccompError::InvalidPolicy);
        }
        clauses.sort_unstable();
        if clauses.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(SeccompError::DuplicateArgumentClause);
        }
        Ok(Self { syscall, clauses })
    }

    pub const fn syscall(&self) -> i64 { self.syscall }

    /// Compatibility accessor for the legacy single-clause representation.
    /// For disjunctive rules, use the clauses() accessor to inspect every clause.
    pub fn predicates(&self) -> &[SeccompArgPredicateV1] {
        self.clauses[0].predicates()
    }

    pub fn clauses(&self) -> &[SeccompSyscallClauseV2] {
        &self.clauses
    }

    pub const fn is_disjunctive(&self) -> bool {
        self.clauses.len() > 1
    }

    pub fn matches(&self, syscall: i64, arguments: &[u64; 6]) -> bool {
        self.syscall == syscall
            && self.clauses.iter().any(|clause| clause.matches(arguments))
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
        let disjunctive = self.rules.iter().any(SeccompSyscallRuleV2::is_disjunctive);
        if disjunctive {
            hasher.update(b"PRISM-SECCOMP-SYSCALL-POLICY-V2-CLAUSES-V1");
        } else {
            hasher.update(b"PRISM-SECCOMP-SYSCALL-POLICY-V2");
        }
        if cfg!(target_endian = "little") {
            hasher.update(b"PRISM-SECCOMP-ENDIAN-LITTLE-V1");
        } else {
            hasher.update(b"PRISM-SECCOMP-ENDIAN-BIG-V1");
        }
        hasher.update(&(self.architecture as u32).to_le_bytes());
        hasher.update(&(self.rules.len() as u32).to_le_bytes());
        for rule in &self.rules {
            hasher.update(&(rule.syscall as u32).to_le_bytes());
            if disjunctive {
                hasher.update(&(rule.clauses.len() as u32).to_le_bytes());
                for clause in &rule.clauses {
                    hasher.update(&(clause.predicates.len() as u32).to_le_bytes());
                    for predicate in &clause.predicates {
                        hasher.update(&(predicate.arg_index as u32).to_le_bytes());
                        hasher.update(&predicate.mask.to_le_bytes());
                        hasher.update(&predicate.value.to_le_bytes());
                        hasher.update(&[match predicate.op {
                            SeccompArgPredicateOpV1::MaskedEqual => 0,
                            SeccompArgPredicateOpV1::MaskedNotEqual => 1,
                        }]);
                    }
                }
            } else {
                hasher.update(&(rule.predicates().len() as u32).to_le_bytes());
                for predicate in rule.predicates() {
                    hasher.update(&(predicate.arg_index as u32).to_le_bytes());
                    hasher.update(&predicate.mask.to_le_bytes());
                    hasher.update(&predicate.value.to_le_bytes());
                    hasher.update(&[match predicate.op {
                        SeccompArgPredicateOpV1::MaskedEqual => 0,
                        SeccompArgPredicateOpV1::MaskedNotEqual => 1,
                    }]);
                }
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
    DuplicateArgumentClause,
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
            Self::DuplicateArgumentClause => f.write_str("seccomp argument clause contains a duplicate"),
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
                    1usize
                        + rule.clauses.iter().map(|clause| {
                            1usize
                                + clause.predicates.iter().map(|predicate| {
                                    usize::from(predicate.mask as u32 != 0)
                                        .saturating_mul(4)
                                        .saturating_add(
                                            usize::from((predicate.mask >> 32) as u32 != 0)
                                                .saturating_mul(4),
                                        )
                                }).sum::<usize>()
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

        fn predicate_instruction_count(predicate: &SeccompArgPredicateV1) -> usize {
            usize::from(predicate.mask as u32 != 0) * 4
                + usize::from((predicate.mask >> 32) as u32 != 0) * 4
        }

        fn clause_instruction_count(clause: &SeccompSyscallClauseV2) -> usize {
            1 + clause
                .predicates
                .iter()
                .map(predicate_instruction_count)
                .sum::<usize>()
        }

        fn rule_body_instruction_count(rule: &SeccompSyscallRuleV2) -> usize {
            rule.clauses
                .iter()
                .map(clause_instruction_count)
                .sum::<usize>()
        }

        for rule in &policy.rules {
            let body_len = rule_body_instruction_count(rule);
            let body_jump = u8::try_from(body_len).map_err(|_| SeccompError::FilterTooLarge)?;
            filter.push(jump_eq(rule.syscall as u32, 0, body_jump));

            if !rule.is_disjunctive() {
                for predicate in rule.predicates() {
                    let base = ARG_BASE + u32::from(predicate.arg_index) * 8;
                    let low_mask = predicate.mask as u32;
                    let low_value = predicate.value as u32;
                    let high_mask = (predicate.mask >> 32) as u32;
                    let high_value = (predicate.value >> 32) as u32;

                    match predicate.op {
                        SeccompArgPredicateOpV1::MaskedEqual => {
                            if low_mask != 0 {
                                filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, base));
                                filter.push(stmt(BPF_ALU | BPF_AND | BPF_K, low_mask));
                                filter.push(jump_eq(low_value, 1, 0));
                                filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | libc::EPERM as u32));
                            }
                            if high_mask != 0 {
                                filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, base + 4));
                                filter.push(stmt(BPF_ALU | BPF_AND | BPF_K, high_mask));
                                filter.push(jump_eq(high_value, 1, 0));
                                filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | libc::EPERM as u32));
                            }
                        }
                        SeccompArgPredicateOpV1::MaskedNotEqual => {
                            if low_mask != 0 && high_mask != 0 {
                                filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, base));
                                filter.push(stmt(BPF_ALU | BPF_AND | BPF_K, low_mask));
                                filter.push(jump_eq(low_value, 0, 4));
                                filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, base + 4));
                                filter.push(stmt(BPF_ALU | BPF_AND | BPF_K, high_mask));
                                filter.push(jump_eq(high_value, 0, 1));
                                filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | libc::EPERM as u32));
                            } else {
                                let (base, mask, value) = if low_mask != 0 {
                                    (base, low_mask, low_value)
                                } else {
                                    (base + 4, high_mask, high_value)
                                };
                                filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, base));
                                filter.push(stmt(BPF_ALU | BPF_AND | BPF_K, mask));
                                filter.push(jump_eq(value, 0, 1));
                                filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_ERRNO | libc::EPERM as u32));
                            }
                        }
                    }
                }

                filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_ALLOW));
            } else {
                for (clause_index, clause) in rule.clauses.iter().enumerate() {
                    for (predicate_index, predicate) in clause.predicates.iter().enumerate() {
                        let later_in_predicates = clause
                            .predicates
                            .iter()
                            .skip(predicate_index + 1)
                            .map(predicate_instruction_count)
                            .sum::<usize>();
                        // From a predicate-failure jump, skip the
                        // remainder of this clause only: the current
                        // predicate's EPERM plus later predicates and this
                        // clause's ALLOW. The next alternative clause must
                        // remain reachable.
                        let clause_mismatch_skip = u8::try_from(
                            later_in_predicates
                                .checked_add(2)
                                .ok_or(SeccompError::FilterTooLarge)?,
                        )
                        .map_err(|_| SeccompError::FilterTooLarge)?;

                        let base = ARG_BASE + u32::from(predicate.arg_index) * 8;
                        let low_mask = predicate.mask as u32;
                        let low_value = predicate.value as u32;
                        let high_mask = (predicate.mask >> 32) as u32;
                        let high_value = (predicate.value >> 32) as u32;

                        match predicate.op {
                            SeccompArgPredicateOpV1::MaskedEqual => {
                                if low_mask != 0 {
                                    let high_tail = usize::from(high_mask != 0) * 4
                                        + later_in_predicates
                                        + 2;
                                    let mismatch_skip = u8::try_from(high_tail)
                                        .map_err(|_| SeccompError::FilterTooLarge)?;
                                    filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, base));
                                    filter.push(stmt(BPF_ALU | BPF_AND | BPF_K, low_mask));
                                    filter.push(jump_eq(low_value, 1, mismatch_skip));
                                    filter.push(stmt(
                                        BPF_RET | BPF_K,
                                        SECCOMP_RET_ERRNO | libc::EPERM as u32,
                                    ));
                                }
                                if high_mask != 0 {
                                    filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, base + 4));
                                    filter.push(stmt(BPF_ALU | BPF_AND | BPF_K, high_mask));
                                    filter.push(jump_eq(high_value, 1, clause_mismatch_skip));
                                    filter.push(stmt(
                                        BPF_RET | BPF_K,
                                        SECCOMP_RET_ERRNO | libc::EPERM as u32,
                                    ));
                                }
                            }
                            SeccompArgPredicateOpV1::MaskedNotEqual => {
                                if low_mask != 0 && high_mask != 0 {
                                    filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, base));
                                    filter.push(stmt(BPF_ALU | BPF_AND | BPF_K, low_mask));
                                    filter.push(jump_eq(low_value, 0, 4));
                                    filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, base + 4));
                                    filter.push(stmt(BPF_ALU | BPF_AND | BPF_K, high_mask));
                                    filter.push(jump_eq(high_value, clause_mismatch_skip, 1));
                                    filter.push(stmt(
                                        BPF_RET | BPF_K,
                                        SECCOMP_RET_ERRNO | libc::EPERM as u32,
                                    ));
                                } else {
                                    let (base, mask, value) = if low_mask != 0 {
                                        (base, low_mask, low_value)
                                    } else {
                                        (base + 4, high_mask, high_value)
                                    };
                                    filter.push(stmt(BPF_LD | BPF_W | BPF_ABS, base));
                                    filter.push(stmt(BPF_ALU | BPF_AND | BPF_K, mask));
                                    filter.push(jump_eq(value, clause_mismatch_skip, 1));
                                    filter.push(stmt(
                                        BPF_RET | BPF_K,
                                        SECCOMP_RET_ERRNO | libc::EPERM as u32,
                                    ));
                                }
                            }
                        }
                    }

                    filter.push(stmt(BPF_RET | BPF_K, SECCOMP_RET_ALLOW));
                }
            }
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
        if policy.rules.iter().any(SeccompSyscallRuleV2::is_disjunctive) {
            hasher.update(b"PRISM-SECCOMP-SYSCALL-EVIDENCE-V5");
            hasher.update(b"PRISM-SECCOMP-POLICY-SCHEMA-V2-CLAUSES-V1");
        } else {
            hasher.update(b"PRISM-SECCOMP-SYSCALL-EVIDENCE-V4");
        }
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
        fn v2_disjunctive_clauses_are_canonical_and_model_is_explicit_or() {
            let arch = SeccompArchitecture::current().unwrap();
            let unix = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_UNIX as u64).unwrap(),
            ])
            .unwrap();
            let netlink = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_NETLINK as u64).unwrap(),
            ])
            .unwrap();

            let rule = SeccompSyscallRuleV2::new_with_clauses(
                libc::SYS_socket,
                vec![netlink.clone(), unix.clone()],
            )
            .unwrap();
            assert_eq!(rule.clauses().len(), 2);
            assert_eq!(rule.clauses()[0], unix);
            assert_eq!(rule.clauses()[1], netlink);

            let policy = SeccompSyscallPolicyV2::new(arch, vec![rule]).unwrap();
            assert!(policy.allows(
                libc::SYS_socket,
                &[libc::AF_UNIX as u64, 1, 0, 0, 0, 0]
            ));
            assert!(policy.allows(
                libc::SYS_socket,
                &[libc::AF_NETLINK as u64, 1, 0, 0, 0, 0]
            ));
            assert!(!policy.allows(
                libc::SYS_socket,
                &[libc::AF_INET as u64, 1, 0, 0, 0, 0]
            ));
            assert!(!policy.allows(libc::SYS_getpid, &[0; 6]));
        }

        #[test]
        fn v2_disjunctive_policy_digest_is_order_independent_and_schema_bound() {
            let arch = SeccompArchitecture::current().unwrap();
            let unix = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_UNIX as u64).unwrap(),
            ])
            .unwrap();
            let netlink = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_NETLINK as u64).unwrap(),
            ])
            .unwrap();

            let forward = SeccompSyscallRuleV2::new_with_clauses(
                libc::SYS_socket,
                vec![unix.clone(), netlink.clone()],
            )
            .unwrap();
            let reverse = SeccompSyscallRuleV2::new_with_clauses(
                libc::SYS_socket,
                vec![netlink, unix],
            )
            .unwrap();
            let first = SeccompSyscallPolicyV2::new(arch, vec![forward]).unwrap();
            let second = SeccompSyscallPolicyV2::new(arch, vec![reverse]).unwrap();
            assert_eq!(first.digest(), second.digest());

            let single = SeccompSyscallRuleV2::new(
                libc::SYS_socket,
                vec![SeccompArgPredicateV1::new(
                    0,
                    u64::MAX,
                    libc::AF_UNIX as u64,
                )
                .unwrap()],
            )
            .unwrap();
            let single_policy =
                SeccompSyscallPolicyV2::new(arch, vec![single]).unwrap();
            assert_ne!(first.digest(), single_policy.digest());
        }

        #[test]
        fn v2_forbidden_renderer_syscall_cannot_be_hidden_in_a_disjunctive_rule() {
            let arch = SeccompArchitecture::current().unwrap();
            let tracer = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, 0).unwrap(),
            ])
            .unwrap();
            let forbidden =
                SeccompSyscallRuleV2::new_with_clauses(libc::SYS_ptrace, vec![tracer])
                    .unwrap();
            let policy = SeccompSyscallPolicyV2::new(arch, vec![forbidden]).unwrap();

            assert!(matches!(
                validate_v2_renderer_policy(&policy),
                Err(SeccompError::ForbiddenRendererSyscall(syscall))
                    if syscall == libc::SYS_ptrace
            ));
        }

        #[test]
        fn v2_disjunctive_clauses_reject_duplicate_and_unconditional_widening() {
            let unix = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_UNIX as u64).unwrap(),
            ])
            .unwrap();
            assert!(matches!(
                SeccompSyscallRuleV2::new_with_clauses(
                    libc::SYS_socket,
                    vec![unix.clone(), unix]
                ),
                Err(SeccompError::DuplicateArgumentClause)
            ));

            let unconditional = SeccompSyscallClauseV2::new(Vec::new()).unwrap();
            let bounded = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_UNIX as u64).unwrap(),
            ])
            .unwrap();
            assert!(matches!(
                SeccompSyscallRuleV2::new_with_clauses(
                    libc::SYS_socket,
                    vec![unconditional, bounded]
                ),
                Err(SeccompError::InvalidPolicy)
            ));
        }

        #[test]
        fn v2_single_clause_digest_is_legacy_compatible_and_disjunctive_is_separated() {
            let arch = SeccompArchitecture::current().unwrap();
            let predicate =
                SeccompArgPredicateV1::new(0, u64::MAX, libc::PR_GET_NO_NEW_PRIVS as u64).unwrap();
            let legacy =
                SeccompSyscallRuleV2::new(libc::SYS_prctl, vec![predicate]).unwrap();
            let clause = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, libc::PR_GET_NO_NEW_PRIVS as u64)
                    .unwrap(),
            ])
            .unwrap();
            let same_semantics = SeccompSyscallRuleV2::new_with_clauses(
                libc::SYS_prctl,
                vec![clause],
            )
            .unwrap();

            let first = SeccompSyscallPolicyV2::new(arch, vec![legacy]).unwrap();
            let second = SeccompSyscallPolicyV2::new(arch, vec![same_semantics]).unwrap();
            assert_eq!(first.digest(), second.digest());

            let unix = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_UNIX as u64).unwrap(),
            ])
            .unwrap();
            let netlink = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_NETLINK as u64).unwrap(),
            ])
            .unwrap();
            let disjunctive = SeccompSyscallRuleV2::new_with_clauses(
                libc::SYS_socket,
                vec![unix, netlink],
            )
            .unwrap();
            let third = SeccompSyscallPolicyV2::new(arch, vec![disjunctive]).unwrap();
            assert_ne!(second.digest(), third.digest());
        }

        #[test]
        fn v2_disjunctive_compiler_jumps_to_next_clause_then_default_deny() {
            let arch = SeccompArchitecture::current().unwrap();
            let unix = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_UNIX as u64).unwrap(),
            ])
            .unwrap();
            let netlink = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_NETLINK as u64).unwrap(),
            ])
            .unwrap();
            let rule = SeccompSyscallRuleV2::new_with_clauses(
                libc::SYS_socket,
                vec![unix, netlink],
            )
            .unwrap();
            let denied = SeccompSyscallRuleV2::new(libc::SYS_getpid, Vec::new()).unwrap();
            let policy = SeccompSyscallPolicyV2::new(arch, vec![rule, denied]).unwrap();
            let filter = compile_filter_v2(&policy).unwrap();

            let socket_dispatch = filter
                .iter()
                .position(|instruction| {
                    instruction.code == BPF_JMP | BPF_JEQ | BPF_K
                        && instruction.k == libc::SYS_socket as u32
                })
                .unwrap();
            assert_eq!(filter[socket_dispatch].jt, 0);
            assert!(filter[socket_dispatch].jf > 1);

            let allow_count = filter
                .iter()
                .filter(|instruction| {
                    instruction.code == BPF_RET | BPF_K
                        && instruction.k == SECCOMP_RET_ALLOW
                })
                .count();
            assert_eq!(allow_count, 3);

            assert_eq!(
                filter.last().map(|instruction| instruction.k),
                Some(SECCOMP_RET_ERRNO | libc::EPERM as u32)
            );
        }

        #[test]
        fn v2_disjunctive_compiler_enforces_instruction_budget_boundary() {
            let arch = SeccompArchitecture::current().unwrap();

            fn full_clause(syscall: i64) -> SeccompSyscallClauseV2 {
                SeccompSyscallClauseV2::new(vec![
                    SeccompArgPredicateV1::new(0, u64::MAX, 0).unwrap(),
                    SeccompArgPredicateV1::new(1, u64::MAX, 1).unwrap(),
                    SeccompArgPredicateV1::new(2, u64::MAX, 2).unwrap(),
                    SeccompArgPredicateV1::new(3, u64::MAX, 3).unwrap(),
                ])
                .unwrap()
            }

            let full_rule = |syscall: i64| {
                SeccompSyscallRuleV2::new_with_clauses(
                    syscall,
                    vec![
                        full_clause(syscall),
                        full_clause(syscall + 1),
                        full_clause(syscall + 2),
                        full_clause(syscall + 3),
                    ],
                )
                .unwrap()
            };

            let under = (0..30)
                .map(|index| full_rule(10_000 + index * 4))
                .collect::<Vec<_>>();
            let under_policy = SeccompSyscallPolicyV2::new(arch, under).unwrap();
            let under_filter = compile_filter_v2(&under_policy).unwrap();
            assert!(under_filter.len() <= 4096);

            let over = (0..31)
                .map(|index| full_rule(20_000 + index * 4))
                .collect::<Vec<_>>();
            let over_policy = SeccompSyscallPolicyV2::new(arch, over).unwrap();
            assert!(matches!(
                compile_filter_v2(&over_policy),
                Err(SeccompError::FilterTooLarge)
            ));
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
        fn v2_masked_not_equal_is_whole_word_not_half_word() {
            let predicate = SeccompArgPredicateV1::new_with_op(
                0,
                u64::MAX,
                0x1122_3344_5566_7788,
                SeccompArgPredicateOpV1::MaskedNotEqual,
            ).unwrap();

            assert!(!predicate.matches(0x1122_3344_5566_7788));
            assert!(predicate.matches(0x1122_3344_5566_7789));
            assert!(predicate.matches(0x1122_3344_5566_0000));
            assert!(predicate.matches(0x0000_0000_5566_7788));
        }

        #[test]
        fn v2_masked_not_equal_compiler_requires_both_halves_to_match_for_denial() {
            let arch = SeccompArchitecture::current().unwrap();
            let rule = SeccompSyscallRuleV2::new(
                libc::SYS_mmap,
                vec![SeccompArgPredicateV1::new_with_op(
                    2,
                    u64::MAX,
                    0x1122_3344_5566_7788,
                    SeccompArgPredicateOpV1::MaskedNotEqual,
                ).unwrap()],
            ).unwrap();
            let policy = SeccompSyscallPolicyV2::new(arch, vec![rule]).unwrap();
            let filter = compile_filter_v2(&policy).unwrap();

            let jump = filter.iter().position(|instruction| {
                instruction.code == BPF_JMP | BPF_JEQ | BPF_K
                    && instruction.k == 0x5566_7788
            }).unwrap();
            // Low-half equality falls through to the high-half load; a low-half
            // mismatch skips exactly the remaining high-half predicate body.
            assert_eq!((filter[jump].jt, filter[jump].jf), (0, 4));
        }

        #[test]
        fn v2_compiler_preserves_predicate_branch_polarity() {
            let arch = SeccompArchitecture::current().unwrap();
            let equal = SeccompSyscallRuleV2::new(
                libc::SYS_prctl,
                vec![SeccompArgPredicateV1::new(0, 0xff, 0x12).unwrap()],
            ).unwrap();
            let not_equal = SeccompSyscallRuleV2::new(
                libc::SYS_ioctl,
                vec![SeccompArgPredicateV1::new_with_op(
                    1,
                    0xff,
                    0x34,
                    SeccompArgPredicateOpV1::MaskedNotEqual,
                ).unwrap()],
            ).unwrap();

            let policy = SeccompSyscallPolicyV2::new(arch, vec![equal, not_equal]).unwrap();
            let filter = compile_filter_v2(&policy).unwrap();

            let prctl = filter.iter().position(|instruction| {
                instruction.code == BPF_JMP | BPF_JEQ | BPF_K
                    && instruction.k == libc::SYS_prctl as u32
            }).unwrap();
            let equal_jump = &filter[prctl + 3];
            assert_eq!((equal_jump.jt, equal_jump.jf), (1, 0));

            let ioctl = filter.iter().position(|instruction| {
                instruction.code == BPF_JMP | BPF_JEQ | BPF_K
                    && instruction.k == libc::SYS_ioctl as u32
            }).unwrap();
            let not_equal_jump = &filter[ioctl + 3];
            assert_eq!((not_equal_jump.jt, not_equal_jump.jf), (0, 1));
        }

        #[test]
        fn v2_compiler_jump_offsets_skip_only_the_current_rule() {
            let arch = SeccompArchitecture::current().unwrap();
            // Policy construction canonicalizes rules by syscall number, so
            // use two ascending syscall numbers to make the expected control
            // flow explicit: socket(2) precedes prctl(2).
            let socket_rule = SeccompSyscallRuleV2::new(
                libc::SYS_socket,
                vec![SeccompArgPredicateV1::new(
                    0,
                    u32::MAX as u64,
                    libc::AF_UNIX as u64,
                ).unwrap()],
            ).unwrap();
            let prctl_rule = SeccompSyscallRuleV2::new(
                libc::SYS_prctl,
                Vec::new(),
            ).unwrap();
            let policy = SeccompSyscallPolicyV2::new(
                arch,
                vec![prctl_rule, socket_rule],
            ).unwrap();
            let filter = compile_filter_v2(&policy).unwrap();

            // The socket dispatch must skip exactly its predicate body plus
            // its ALLOW and land on the next syscall comparison. This proves
            // the mismatch jump is measured from the instruction after the
            // dispatch comparison, not from the comparison itself.
            let socket_index = filter.iter().position(|instruction| {
                instruction.code == BPF_JMP | BPF_JEQ | BPF_K
                    && instruction.k == libc::SYS_socket as u32
            }).unwrap();
            let prctl_offset = filter[socket_index + 1..].iter().position(|instruction| {
                instruction.code == BPF_JMP | BPF_JEQ | BPF_K
                    && instruction.k == libc::SYS_prctl as u32
            }).unwrap();
            assert_eq!(filter[socket_index].jf as usize, prctl_offset);

            let prctl_index = socket_index + 1 + prctl_offset;
            assert_eq!(filter[prctl_index].k, libc::SYS_prctl as u32);
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

        fn interpret_v2_filter(
            filter: &[SockFilter],
            architecture: SeccompArchitecture,
            syscall: i64,
            arguments: [u64; 6],
        ) -> u32 {
            let mut accumulator = 0u32;
            let mut pc = 0usize;

            for _ in 0..=filter.len() {
                let instruction = filter
                    .get(pc)
                    .unwrap_or_else(|| panic!("BPF program fell off the end at {pc}"));

                match instruction.code {
                    code if code == BPF_LD | BPF_W | BPF_ABS => {
                        accumulator = match instruction.k {
                            SECCOMP_DATA_ARCH_OFFSET => architecture as u32,
                            SECCOMP_DATA_NR_OFFSET => syscall as u32,
                            offset if (16..=56).contains(&offset) && (offset - 16) % 8 == 0 => {
                                let index = ((offset - 16) / 8) as usize;
                                arguments[index] as u32
                            }
                            offset
                                if (20..=60).contains(&offset) && (offset - 20) % 8 == 0 =>
                            {
                                let index = ((offset - 20) / 8) as usize;
                                (arguments[index] >> 32) as u32
                            }
                            offset => panic!("unexpected BPF argument offset {offset}"),
                        };
                        pc += 1;
                    }
                    code if code == BPF_ALU | BPF_AND | BPF_K => {
                        accumulator &= instruction.k;
                        pc += 1;
                    }
                    code if code == BPF_JMP | BPF_JEQ | BPF_K => {
                        pc += 1 + if accumulator == instruction.k {
                            instruction.jt as usize
                        } else {
                            instruction.jf as usize
                        };
                    }
                    code if code == BPF_JMP | BPF_JGE | BPF_K => {
                        pc += 1 + if accumulator >= instruction.k {
                            instruction.jt as usize
                        } else {
                            instruction.jf as usize
                        };
                    }
                    code if code == BPF_RET | BPF_K => return instruction.k,
                    code => panic!("unexpected V2 BPF opcode 0x{code:04x}"),
                }
            }

            panic!("BPF program exceeded the execution bound");
        }

        #[test]
        fn v2_compiled_filter_matches_policy_model_for_adversarial_arguments() {
            let arch = SeccompArchitecture::current().unwrap();
            let prctl = SeccompSyscallRuleV2::new(
                libc::SYS_prctl,
                vec![SeccompArgPredicateV1::new(
                    0,
                    u64::MAX,
                    libc::PR_GET_NO_NEW_PRIVS as u64,
                )
                .unwrap()],
            )
            .unwrap();
            let mmap_no_wx = SeccompSyscallRuleV2::new(
                libc::SYS_mmap,
                vec![SeccompArgPredicateV1::new_with_op(
                    2,
                    (libc::PROT_WRITE | libc::PROT_EXEC) as u64,
                    (libc::PROT_WRITE | libc::PROT_EXEC) as u64,
                    SeccompArgPredicateOpV1::MaskedNotEqual,
                )
                .unwrap()],
            )
            .unwrap();
            let mprotect_no_x = SeccompSyscallRuleV2::new(
                libc::SYS_mprotect,
                vec![SeccompArgPredicateV1::new_with_op(
                    2,
                    libc::PROT_EXEC as u64,
                    libc::PROT_EXEC as u64,
                    SeccompArgPredicateOpV1::MaskedNotEqual,
                )
                .unwrap()],
            )
            .unwrap();
            let exit_group =
                SeccompSyscallRuleV2::new(libc::SYS_exit_group, Vec::new()).unwrap();
            let policy =
                SeccompSyscallPolicyV2::new(arch, vec![prctl, mmap_no_wx, mprotect_no_x, exit_group])
                    .unwrap();
            let filter = compile_filter_v2(&policy).unwrap();

            let cases = [
                (
                    libc::SYS_prctl,
                    [libc::PR_GET_NO_NEW_PRIVS as u64, 0, 0, 0, 0, 0],
                    SECCOMP_RET_ALLOW,
                ),
                (
                    libc::SYS_prctl,
                    [libc::PR_SET_NO_NEW_PRIVS as u64, 1, 0, 0, 0, 0],
                    SECCOMP_RET_ERRNO | libc::EPERM as u32,
                ),
                (
                    libc::SYS_mmap,
                    [
                        0,
                        4096,
                        (libc::PROT_READ | libc::PROT_WRITE) as u64,
                        (libc::MAP_PRIVATE | libc::MAP_ANONYMOUS) as u64,
                        u64::MAX,
                        0,
                    ],
                    SECCOMP_RET_ALLOW,
                ),
                (
                    libc::SYS_mmap,
                    [
                        0,
                        4096,
                        (libc::PROT_READ | libc::PROT_WRITE | libc::PROT_EXEC) as u64,
                        (libc::MAP_PRIVATE | libc::MAP_ANONYMOUS) as u64,
                        u64::MAX,
                        0,
                    ],
                    SECCOMP_RET_ERRNO | libc::EPERM as u32,
                ),
                (
                    libc::SYS_mmap,
                    [
                        0,
                        4096,
                        (libc::PROT_WRITE | libc::PROT_EXEC) as u64,
                        (libc::MAP_PRIVATE | libc::MAP_ANONYMOUS) as u64,
                        u64::MAX,
                        0,
                    ],
                    SECCOMP_RET_ERRNO | libc::EPERM as u32,
                ),
                (
                    libc::SYS_mprotect,
                    [
                        0,
                        4096,
                        (libc::PROT_READ | libc::PROT_WRITE) as u64,
                        0,
                        0,
                        0,
                    ],
                    SECCOMP_RET_ALLOW,
                ),
                (
                    libc::SYS_mprotect,
                    [
                        0,
                        4096,
                        (libc::PROT_READ | libc::PROT_EXEC) as u64,
                        0,
                        0,
                        0,
                    ],
                    SECCOMP_RET_ERRNO | libc::EPERM as u32,
                ),
                (
                    libc::SYS_getpid,
                    [0; 6],
                    SECCOMP_RET_ERRNO | libc::EPERM as u32,
                ),
                (
                    libc::SYS_exit_group,
                    [0; 6],
                    SECCOMP_RET_ALLOW,
                ),
            ];

            for (syscall, arguments, expected) in cases {
                assert_eq!(
                    policy.allows(syscall, &arguments),
                    expected == SECCOMP_RET_ALLOW,
                    "policy model mismatch for syscall {syscall} args {arguments:?}"
                );
                assert_eq!(
                    interpret_v2_filter(&filter, arch, syscall, arguments),
                    expected,
                    "compiled BPF mismatch for syscall {syscall} args {arguments:?}"
                );
            }

            if arch == SeccompArchitecture::X86_64 {
                assert_eq!(
                    interpret_v2_filter(&filter, arch, 0x4000_0000, [0; 6]),
                    SECCOMP_RET_KILL_PROCESS,
                    "x32 ABI must never fall through to the allowlist"
                );
            }

            let wrong_arch = match arch {
                SeccompArchitecture::X86_64 => SeccompArchitecture::Aarch64,
                SeccompArchitecture::Aarch64 => SeccompArchitecture::X86_64,
                SeccompArchitecture::Riscv64 => SeccompArchitecture::X86_64,
            };
            assert_eq!(
                interpret_v2_filter(&filter, wrong_arch, libc::SYS_exit_group, [0; 6]),
                SECCOMP_RET_KILL_PROCESS,
                "architecture mismatch must kill before syscall dispatch"
            );
        }

        #[test]
        fn v2_disjunctive_64bit_equality_failures_reach_later_clause() {
            let arch = SeccompArchitecture::current().unwrap();

            let first_low_mismatch = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, 0x0000_0001_0000_0001).unwrap(),
            ]).unwrap();
            let second_low_mismatch = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, 0x0000_0002_0000_0002).unwrap(),
            ]).unwrap();
            let low_policy = SeccompSyscallPolicyV2::new(
                arch,
                vec![SeccompSyscallRuleV2::new_with_clauses(
                    libc::SYS_socket,
                    vec![first_low_mismatch, second_low_mismatch],
                ).unwrap()],
            ).unwrap();
            let low_filter = compile_filter_v2(&low_policy).unwrap();

            let low_args = [0x0000_0002_0000_0002, 0, 0, 0, 0, 0];
            assert!(low_policy.allows(libc::SYS_socket, &low_args));
            assert_eq!(
                interpret_v2_filter(&low_filter, arch, libc::SYS_socket, low_args),
                SECCOMP_RET_ALLOW,
                "low-word predicate failure must fall through to the next clause"
            );

            let first_high_mismatch = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, 0x0000_0001_0000_0000).unwrap(),
            ]).unwrap();
            let second_high_mismatch = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, 0x0000_0002_0000_0000).unwrap(),
            ]).unwrap();
            let high_policy = SeccompSyscallPolicyV2::new(
                arch,
                vec![SeccompSyscallRuleV2::new_with_clauses(
                    libc::SYS_socket,
                    vec![first_high_mismatch, second_high_mismatch],
                ).unwrap()],
            ).unwrap();
            let high_filter = compile_filter_v2(&high_policy).unwrap();

            let high_args = [0x0000_0002_0000_0000, 0, 0, 0, 0, 0];
            assert!(high_policy.allows(libc::SYS_socket, &high_args));
            assert_eq!(
                interpret_v2_filter(&high_filter, arch, libc::SYS_socket, high_args),
                SECCOMP_RET_ALLOW,
                "high-word predicate failure must fall through to the next clause"
            );
        }

        #[test]
        fn v2_disjunctive_masked_not_equal_failure_reaches_later_clause() {
            let arch = SeccompArchitecture::current().unwrap();
            let not_equal = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new_with_op(
                    0,
                    0x0000_0000_0000_00ff,
                    1,
                    SeccompArgPredicateOpV1::MaskedNotEqual,
                ).unwrap(),
            ]).unwrap();
            let later_equal = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(
                    0,
                    0x0000_0000_0000_ffff,
                    1,
                ).unwrap(),
            ]).unwrap();
            let rule = SeccompSyscallRuleV2::new_with_clauses(
                libc::SYS_socket,
                vec![not_equal, later_equal],
            ).unwrap();
            assert_eq!(rule.clauses()[0].predicates()[0].op(), SeccompArgPredicateOpV1::MaskedNotEqual);

            let policy = SeccompSyscallPolicyV2::new(arch, vec![rule]).unwrap();
            let filter = compile_filter_v2(&policy).unwrap();
            let args = [1, 0, 0, 0, 0, 0];

            assert!(policy.allows(libc::SYS_socket, &args));
            assert_eq!(
                interpret_v2_filter(&filter, arch, libc::SYS_socket, args),
                SECCOMP_RET_ALLOW,
                "masked-not-equal failure must fall through to the next clause"
            );
        }

        #[test]
        fn v2_disjunctive_compiled_filter_matches_model_across_clause_boundaries() {
            let arch = SeccompArchitecture::current().unwrap();
            let unix = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_UNIX as u64).unwrap(),
            ])
            .unwrap();
            let netlink = SeccompSyscallClauseV2::new(vec![
                SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_NETLINK as u64).unwrap(),
            ])
            .unwrap();
            let rule = SeccompSyscallRuleV2::new_with_clauses(
                libc::SYS_socket,
                vec![unix, netlink],
            )
            .unwrap();
            let policy = SeccompSyscallPolicyV2::new(arch, vec![rule]).unwrap();
            let filter = compile_filter_v2(&policy).unwrap();

            for domain in [libc::AF_UNIX, libc::AF_NETLINK] {
                let args = [domain as u64, 1, 0, 0, 0, 0];
                assert_eq!(policy.allows(libc::SYS_socket, &args), true);
                assert_eq!(
                    interpret_v2_filter(&filter, arch, libc::SYS_socket, args),
                    SECCOMP_RET_ALLOW,
                    "allowed clause {domain} must reach ALLOW"
                );
            }

            let denied_args = [libc::AF_INET as u64, 1, 0, 0, 0, 0];
            assert!(!policy.allows(libc::SYS_socket, &denied_args));
            assert_eq!(
                interpret_v2_filter(&filter, arch, libc::SYS_socket, denied_args),
                SECCOMP_RET_ERRNO | libc::EPERM as u32,
                "failed alternatives must reach global default deny"
            );
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
