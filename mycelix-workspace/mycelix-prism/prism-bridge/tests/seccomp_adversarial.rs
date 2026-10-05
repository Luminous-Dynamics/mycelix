//! Hosted adversarial seccomp checks.
//!
//! The filter is irreversible for the calling process, so the enforcement
//! test executes the installation in a dedicated child test process. The
//! parent test runner remains unsandboxed.

#[cfg(target_os = "linux")]
fn child() -> ! {
    use prism_bridge::process::{RendererProcessAssignmentId, SandboxProfileV1};
    use prism_bridge::seccomp::{install, SeccompArchitecture, SeccompSyscallPolicyV1};

    let architecture = SeccompArchitecture::current().unwrap_or_else(|| unsafe {
        libc::_exit(90)
    });

    let policy = SeccompSyscallPolicyV1::new(
        architecture,
        vec![libc::SYS_getpid, libc::SYS_write, libc::SYS_exit_group],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(91) });
    let profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(policy.digest())
        .unwrap_or_else(|_| unsafe { libc::_exit(92) });

    if install(
        RendererProcessAssignmentId::new(1).unwrap(),
        profile,
        &policy,
    )
    .is_err()
    {
        unsafe { libc::_exit(92) };
    }

    // Cross the libc boundary for the allowed path as well. A successful
    // libc getpid() alone does not prove that the kernel evaluated the
    // syscall against this filter.
    if unsafe { libc::syscall(libc::SYS_getpid) } <= 0 {
        unsafe { libc::_exit(93) };
    }

    // Exercise the kernel syscall directly rather than the libc getppid()
    // wrapper: getppid() is specified as always-successful, so its wrapper
    // behavior is not the right boundary for proving a seccomp errno action.
    let denied = unsafe { libc::syscall(libc::SYS_getppid) };
    let errno = std::io::Error::last_os_error().raw_os_error();
    if denied != -1 || errno != Some(libc::EPERM) {
        unsafe { libc::_exit(94) };
    }

    // This syscall is normally capable of reading memory from another
    // process. Use a self-read with a valid address so that, without the
    // seccomp filter, the operation has a successful kernel path. After
    // installation the renderer policy must reject it with the default EPERM.
    let mut byte = 0u8;
    let local = libc::iovec {
        iov_base: (&mut byte as *mut u8).cast(),
        iov_len: 1,
    };
    let remote = libc::iovec {
        iov_base: (&byte as *const u8).cast_mut().cast(),
        iov_len: 1,
    };
    let vm_read = unsafe {
        libc::syscall(
            libc::SYS_process_vm_readv,
            libc::syscall(libc::SYS_getpid),
            &local as *const libc::iovec,
            1usize,
            &remote as *const libc::iovec,
            1usize,
            0usize,
        )
    };
    let vm_errno = std::io::Error::last_os_error().raw_os_error();
    if vm_read != -1 || vm_errno != Some(libc::EPERM) {
        unsafe { libc::_exit(95) };
    }

    // Exercise the corresponding write-side primitive against a valid
    // writable self-address. Without the filter this has a successful kernel
    // path; with renderer policy it must be denied with EPERM.
    let write_byte = byte;
    let write_local = libc::iovec {
        iov_base: (&write_byte as *const u8).cast_mut().cast(),
        iov_len: 1,
    };
    let write_remote = libc::iovec {
        iov_base: (&mut byte as *mut u8).cast(),
        iov_len: 1,
    };
    let vm_write = unsafe {
        libc::syscall(
            libc::SYS_process_vm_writev,
            libc::syscall(libc::SYS_getpid),
            &write_local as *const libc::iovec,
            1usize,
            &write_remote as *const libc::iovec,
            1usize,
            0usize,
        )
    };
    let vm_write_errno = std::io::Error::last_os_error().raw_os_error();
    if vm_write != -1 || vm_write_errno != Some(libc::EPERM) {
        unsafe { libc::_exit(96) };
    }

    unsafe { libc::_exit(0) }
}

#[cfg(target_os = "linux")]
#[test]
fn seccomp_enforcement_is_isolated_and_fail_closed() {
    if std::env::var_os("PRISM_SECCOMP_CHILD").is_some() {
        child();
    }

    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .arg("--exact")
        .arg("seccomp_enforcement_is_isolated_and_fail_closed")
        .arg("--nocapture")
        .env("PRISM_SECCOMP_CHILD", "1")
        .status()
        .expect("failed to launch seccomp child");

    assert!(status.success(), "seccomp child failed: {status}");
}

#[cfg(not(target_os = "linux"))]
#[test]
fn seccomp_enforcement_is_not_claimed_off_linux() {
    // The adapter is Linux-specific; non-Linux CI remains green without
    // pretending that Linux kernel enforcement was exercised.
}

#[cfg(target_os = "linux")]
fn parameter_predicate_child() -> ! {
    use prism_bridge::process::{RendererProcessAssignmentId, SandboxProfileV1};
    use prism_bridge::seccomp::{
        install_v2, SeccompArgPredicateV1, SeccompArchitecture, SeccompSyscallPolicyV2,
        SeccompSyscallRuleV2,
    };

    let architecture =
        SeccompArchitecture::current().unwrap_or_else(|| unsafe { libc::_exit(110) });

    let prctl_get = SeccompSyscallRuleV2::new(
        libc::SYS_prctl,
        vec![SeccompArgPredicateV1::new(
            0,
            u64::MAX,
            libc::PR_GET_NO_NEW_PRIVS as u64,
        )
        .unwrap_or_else(|_| unsafe { libc::_exit(111) })],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(112) });
    let mmap_no_wx = SeccompSyscallRuleV2::new(
        libc::SYS_mmap,
        vec![SeccompArgPredicateV1::new_with_op(
            2,
            (libc::PROT_WRITE | libc::PROT_EXEC) as u64,
            (libc::PROT_WRITE | libc::PROT_EXEC) as u64,
            prism_bridge::seccomp::SeccompArgPredicateOpV1::MaskedNotEqual,
        )
        .unwrap_or_else(|_| unsafe { libc::_exit(113) })],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(114) });
    let mprotect_no_x = SeccompSyscallRuleV2::new(
        libc::SYS_mprotect,
        vec![SeccompArgPredicateV1::new_with_op(
            2,
            libc::PROT_EXEC as u64,
            libc::PROT_EXEC as u64,
            prism_bridge::seccomp::SeccompArgPredicateOpV1::MaskedNotEqual,
        )
        .unwrap_or_else(|_| unsafe { libc::_exit(115) })],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(116) });
    let exit_group = SeccompSyscallRuleV2::new(libc::SYS_exit_group, Vec::new())
        .unwrap_or_else(|_| unsafe { libc::_exit(117) });

    let policy = SeccompSyscallPolicyV2::new(
        architecture,
        vec![prctl_get, mmap_no_wx, mprotect_no_x, exit_group],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(118) });
    let profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(policy.digest())
        .unwrap_or_else(|_| unsafe { libc::_exit(119) });

    // Warm the raw syscall/errno path before the irreversible transition.
    let _ = unsafe {
        libc::syscall(
            libc::SYS_prctl,
            libc::PR_GET_NO_NEW_PRIVS,
            0,
            0,
            0,
            0,
        )
    };
    let _ = unsafe { *libc::__errno_location() };

    if install_v2(
        RendererProcessAssignmentId::new(4).unwrap(),
        profile,
        &policy,
    )
    .is_err()
    {
        unsafe { libc::_exit(120) };
    }

    let allowed = unsafe {
        libc::syscall(
            libc::SYS_prctl,
            libc::PR_GET_NO_NEW_PRIVS,
            0,
            0,
            0,
            0,
        )
    };
    if allowed != 1 {
        unsafe { libc::_exit(121) };
    }

    let read_mapping = unsafe {
        libc::syscall(
            libc::SYS_mmap,
            0usize,
            4096usize,
            libc::PROT_READ,
            libc::MAP_PRIVATE | libc::MAP_ANONYMOUS,
            usize::MAX,
            0usize,
        )
    };
    if read_mapping == -1 {
        unsafe { libc::_exit(122) };
    }

    let wx_mapping = unsafe {
        libc::syscall(
            libc::SYS_mmap,
            0usize,
            4096usize,
            libc::PROT_READ | libc::PROT_WRITE | libc::PROT_EXEC,
            libc::MAP_PRIVATE | libc::MAP_ANONYMOUS,
            usize::MAX,
            0usize,
        )
    };
    let wx_errno = unsafe { *libc::__errno_location() };
    if wx_mapping != -1 || wx_errno != libc::EPERM {
        unsafe { libc::_exit(123) };
    }

    if unsafe {
        libc::syscall(
            libc::SYS_mprotect,
            read_mapping as usize,
            4096usize,
            libc::PROT_READ | libc::PROT_WRITE,
        )
    } != 0
    {
        unsafe { libc::_exit(124) };
    }

    let exec_rc = unsafe {
        libc::syscall(
            libc::SYS_mprotect,
            read_mapping as usize,
            4096usize,
            libc::PROT_READ | libc::PROT_EXEC,
        )
    };
    let exec_errno = unsafe { *libc::__errno_location() };
    if exec_rc != -1 || exec_errno != libc::EPERM {
        unsafe { libc::_exit(125) };
    }

    // Same syscall number, deliberately different first argument: V2 must
    // reject it instead of widening the rule to all prctl invocations.
    let denied = unsafe {
        libc::syscall(
            libc::SYS_prctl,
            libc::PR_SET_NO_NEW_PRIVS,
            1,
            0,
            0,
            0,
        )
    };
    let errno = unsafe { *libc::__errno_location() };
    if denied != -1 || errno != libc::EPERM {
        unsafe { libc::_exit(126) };
    }

    unsafe { libc::_exit(0) }
}

#[cfg(target_os = "linux")]
#[test]
fn seccomp_parameter_predicate_is_positive_and_negative() {
    if std::env::var_os("PRISM_SECCOMP_PARAMETER_CHILD").is_some() {
        parameter_predicate_child();
    }

    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .arg("--exact")
        .arg("seccomp_parameter_predicate_is_positive_and_negative")
        .arg("--nocapture")
        .env("PRISM_SECCOMP_PARAMETER_CHILD", "1")
        .status()
        .expect("failed to launch seccomp parameter child");

    assert!(status.success(), "seccomp parameter child failed: {status}");
}

#[cfg(target_os = "linux")]
fn disjunctive_stage(tag: &[u8]) {
    let _ = unsafe {
        libc::syscall(
            libc::SYS_write,
            libc::STDERR_FILENO,
            tag.as_ptr(),
            tag.len(),
        )
    };
}

#[cfg(target_os = "linux")]
fn disjunctive_socket_child() -> ! {
    use prism_bridge::process::{RendererProcessAssignmentId, SandboxProfileV1};
    use prism_bridge::seccomp::{
        install_v2, SeccompArgPredicateOpV1, SeccompArgPredicateV1, SeccompArchitecture,
        SeccompSyscallClauseV2, SeccompSyscallPolicyV2, SeccompSyscallRuleV2,
    };

    let architecture =
        SeccompArchitecture::current().unwrap_or_else(|| unsafe { libc::_exit(130) });

    // Each alternative is a conjunction: family + exact socket type must both
    // match before that clause's ALLOW is reachable. The two clauses then form
    // the explicit OR over the permitted (family, type) pairs.
    let unix = SeccompSyscallClauseV2::new(vec![
        SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_UNIX as u64)
            .unwrap_or_else(|_| unsafe { libc::_exit(131) }),
        SeccompArgPredicateV1::new(1, u64::MAX, libc::SOCK_STREAM as u64)
            .unwrap_or_else(|_| unsafe { libc::_exit(132) }),
    ])
    .unwrap_or_else(|_| unsafe { libc::_exit(133) });
    let netlink = SeccompSyscallClauseV2::new(vec![
        SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_NETLINK as u64)
            .unwrap_or_else(|_| unsafe { libc::_exit(134) }),
        SeccompArgPredicateV1::new(1, u64::MAX, libc::SOCK_DGRAM as u64)
            .unwrap_or_else(|_| unsafe { libc::_exit(135) }),
    ])
    .unwrap_or_else(|_| unsafe { libc::_exit(136) });
    let socket = SeccompSyscallRuleV2::new_with_clauses(
        libc::SYS_socket,
        vec![unix, netlink],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(135) });
    let lseek_not_equal = SeccompSyscallClauseV2::new(vec![
        SeccompArgPredicateV1::new_with_op(
            1,
            u64::MAX,
            0x0000_0001_0000_0001,
            SeccompArgPredicateOpV1::MaskedNotEqual,
        )
        .unwrap_or_else(|_| unsafe { libc::_exit(136) }),
    ])
    .unwrap_or_else(|_| unsafe { libc::_exit(137) });
    let lseek_equal = SeccompSyscallClauseV2::new(vec![
        SeccompArgPredicateV1::new(
            1,
            u64::MAX,
            0x0000_0001_0000_0001,
        )
        .unwrap_or_else(|_| unsafe { libc::_exit(138) }),
    ])
    .unwrap_or_else(|_| unsafe { libc::_exit(139) });
    let lseek = SeccompSyscallRuleV2::new_with_clauses(
        libc::SYS_lseek,
        vec![lseek_not_equal, lseek_equal],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(140) });

    let write = SeccompSyscallRuleV2::new(libc::SYS_write, Vec::new())
        .unwrap_or_else(|_| unsafe { libc::_exit(141) });
    let exit_group = SeccompSyscallRuleV2::new(libc::SYS_exit_group, Vec::new())
        .unwrap_or_else(|_| unsafe { libc::_exit(142) });

    // Open a stable fd before the filter is installed. /dev/null lseek is
    // harmless; the important observation after installation is whether the
    // syscall reaches the kernel (any non-EPERM result) or is denied by the
    // seccomp filter itself.
    disjunctive_stage(b"A-pre-open\n");
    let seek_fd = unsafe {
        libc::open(
            c"/dev/null".as_ptr(),
            libc::O_RDONLY,
        )
    };
    if seek_fd < 0 {
        unsafe { libc::_exit(142) };
    }

    let policy = SeccompSyscallPolicyV2::new(
        architecture,
        vec![socket, lseek, write, exit_group],
    )
        .unwrap_or_else(|_| unsafe { libc::_exit(137) });
    let profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(policy.digest())
        .unwrap_or_else(|_| unsafe { libc::_exit(138) });

    // Resolve the raw syscall/errno path before the irreversible transition.
    let warmup = unsafe {
        libc::syscall(
            libc::SYS_socket,
            libc::AF_UNIX,
            libc::SOCK_STREAM,
            0,
        )
    };
    if warmup >= 0 {
        unsafe { libc::close(warmup as libc::c_int) };
    }
    let _ = unsafe { *libc::__errno_location() };

    disjunctive_stage(b"B-pre-install\n");
    if install_v2(
        RendererProcessAssignmentId::new(5).unwrap(),
        profile,
        &policy,
    )
    .is_err()
    {
        unsafe { libc::_exit(143) };
    }

    disjunctive_stage(b"C-post-install\n");
    let high_mismatch = unsafe {
        libc::syscall(
            libc::SYS_lseek,
            seek_fd,
            0x0000_0002_0000_0001u64,
            libc::SEEK_SET,
        )
    };
    let high_mismatch_errno = unsafe { *libc::__errno_location() };
    if high_mismatch == -1 && high_mismatch_errno == libc::EPERM {
        unsafe { libc::_exit(144) };
    }

    disjunctive_stage(b"D-high-mismatch-ok\n");
    let exact_match = unsafe {
        libc::syscall(
            libc::SYS_lseek,
            seek_fd,
            0x0000_0001_0000_0001u64,
            libc::SEEK_SET,
        )
    };
    let exact_match_errno = unsafe { *libc::__errno_location() };
    if exact_match == -1 && exact_match_errno == libc::EPERM {
        unsafe { libc::_exit(145) };
    }

    disjunctive_stage(b"E-exact-match-ok\n");
    let unix_socket =
        unsafe { libc::syscall(libc::SYS_socket, libc::AF_UNIX, libc::SOCK_STREAM, 0) };
    if unix_socket < 0 {
        unsafe { libc::_exit(140) };
    }

    disjunctive_stage(b"F-unix-ok\n");
    let netlink_socket =
        unsafe { libc::syscall(libc::SYS_socket, libc::AF_NETLINK, libc::SOCK_DGRAM, 0) };
    if netlink_socket < 0 {
        unsafe { libc::_exit(141) };
    }

    // First-predicate match + second-predicate mismatch must still deny.
    let unix_wrong_type = unsafe {
        libc::syscall(libc::SYS_socket, libc::AF_UNIX, libc::SOCK_DGRAM, 0)
    };
    let unix_wrong_type_errno = unsafe { *libc::__errno_location() };
    if unix_wrong_type != -1 || unix_wrong_type_errno != libc::EPERM {
        unsafe { libc::_exit(142) };
    }

    // Second clause's first predicate matches, but its type predicate fails.
    let netlink_wrong_type = unsafe {
        libc::syscall(libc::SYS_socket, libc::AF_NETLINK, libc::SOCK_STREAM, 0)
    };
    let netlink_wrong_type_errno = unsafe { *libc::__errno_location() };
    if netlink_wrong_type != -1 || netlink_wrong_type_errno != libc::EPERM {
        unsafe { libc::_exit(143) };
    }

    let denied = unsafe {
        libc::syscall(
            libc::SYS_socket,
            libc::AF_INET,
            libc::SOCK_STREAM,
            0,
        )
    };
    let denied_errno = unsafe { *libc::__errno_location() };
    if denied != -1 || denied_errno != libc::EPERM {
        unsafe { libc::_exit(144) };
    }

    disjunctive_stage(b"H-invalid-pairs-denied-ok\n");
    let unlisted = unsafe { libc::syscall(libc::SYS_getpid) };
    let unlisted_errno = unsafe { *libc::__errno_location() };
    if unlisted != -1 || unlisted_errno != libc::EPERM {
        unsafe { libc::_exit(143) };
    }

    disjunctive_stage(b"I-getpid-denied-ok\n");
    unsafe { libc::_exit(0) }
}

#[cfg(target_os = "linux")]
#[test]
fn seccomp_disjunctive_argument_clauses_are_enforced() {
    if std::env::var_os("PRISM_SECCOMP_DISJUNCTIVE_CHILD").is_some() {
        disjunctive_socket_child();
    }

    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .arg("--exact")
        .arg("seccomp_disjunctive_argument_clauses_are_enforced")
        .arg("--nocapture")
        .env("RUST_TEST_THREADS", "1")
        .env("PRISM_SECCOMP_DISJUNCTIVE_CHILD", "1")
        .status()
        .expect("failed to launch seccomp disjunctive child");

    assert!(status.success(), "seccomp disjunctive child failed: {status}");
}

#[cfg(target_os = "linux")]
fn maximum_dispatch_offset_child() -> ! {
    use prism_bridge::process::{RendererProcessAssignmentId, SandboxProfileV1};
    use prism_bridge::seccomp::{
        install_v2, SeccompArgPredicateOpV1, SeccompArgPredicateV1, SeccompArchitecture,
        SeccompSyscallClauseV2, SeccompSyscallPolicyV2, SeccompSyscallRuleV2,
    };

    let architecture =
        SeccompArchitecture::current().unwrap_or_else(|| unsafe { libc::_exit(150) });

    // Four clauses x four full-width MaskedNotEqual predicates is the
    // compiler's maximum V2 disjunctive rule-body shape:
    // 4 * (4 * 6 + 1) = 100 instructions. Make read() the first rule and
    // write() the following unconditional rule. read < write < exit_group
    // on all three supported Linux architectures, so the write() probe must
    // take a 100-instruction forward jump. The final unconditional
    // exit_group rule keeps the child termination path explicit and fail-closed.
    let mut clauses = Vec::with_capacity(4);
    for clause_index in 0..4u64 {
        let mut predicates = Vec::with_capacity(4);
        for arg_index in 0..4u8 {
            let value = 0x0100_0000_0000_0000u64
                | (clause_index << 12)
                | u64::from(arg_index);
            let predicate = SeccompArgPredicateV1::new_with_op(
                arg_index,
                u64::MAX,
                value,
                SeccompArgPredicateOpV1::MaskedNotEqual,
            )
            .unwrap_or_else(|_| unsafe { libc::_exit(151) });
            predicates.push(predicate);
        }
        clauses.push(
            SeccompSyscallClauseV2::new(predicates)
                .unwrap_or_else(|_| unsafe { libc::_exit(152) }),
        );
    }

    let mut dispatch_chain = [libc::SYS_read, libc::SYS_write, libc::SYS_exit_group];
    dispatch_chain.sort_unstable();
    let large_dispatch_syscall = dispatch_chain[0];
    let allowed_syscall = dispatch_chain[1];
    let cleanup_syscall = dispatch_chain[2];

    let large_rule = SeccompSyscallRuleV2::new_with_clauses(large_dispatch_syscall, clauses)
        .unwrap_or_else(|_| unsafe { libc::_exit(154) });
    let unconditional_allowed = SeccompSyscallRuleV2::new(allowed_syscall, Vec::new())
        .unwrap_or_else(|_| unsafe { libc::_exit(155) });
    let unconditional_exit = SeccompSyscallRuleV2::new(cleanup_syscall, Vec::new())
        .unwrap_or_else(|_| unsafe { libc::_exit(156) });

    // The first installed filter must also permit the syscalls needed to
    // layer additional filters and to service the Rust allocator while the
    // cumulative-path-limit fixture is running.
    let runtime_syscalls = [
        libc::SYS_close,
        libc::SYS_mmap,
        libc::SYS_mprotect,
        libc::SYS_munmap,
        libc::SYS_brk,
        libc::SYS_getrandom,
        libc::SYS_prctl,
        libc::SYS_seccomp,
        libc::SYS_getppid,
    ];
    let runtime_rules = runtime_syscalls
        .into_iter()
        .map(|syscall| {
            SeccompSyscallRuleV2::new(syscall, Vec::new())
                .unwrap_or_else(|_| unsafe { libc::_exit(157) })
        })
        .collect::<Vec<_>>();

    let mut base_rules = vec![large_rule, unconditional_allowed, unconditional_exit];
    base_rules.extend(runtime_rules);
    let policy = SeccompSyscallPolicyV2::new(architecture, base_rules)
        .unwrap_or_else(|_| unsafe { libc::_exit(158) });
    let profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(policy.digest())
        .unwrap_or_else(|_| unsafe { libc::_exit(159) });

    // Build the cumulative-limit policies before installing the first filter.
    // Each has forty maximum-width disjunctive dummy rules plus twelve
    // unconditional runtime rules. Each candidate remains below the single
    // BPF_MAXINSNS bound while repeated attachment approaches the much larger
    // MAX_INSNS_PER_PATH bound.
    //
    // The final candidate swaps getppid for getuid. If the kernel incorrectly
    // attached that candidate despite returning ENOMEM, the post-failure
    // getppid() probe below would be denied by the new filter.
    fn build_cumulative_policy(
        architecture: SeccompArchitecture,
        include_getppid: bool,
    ) -> Result<SeccompSyscallPolicyV2, prism_bridge::seccomp::SeccompError> {
        let mut rules = Vec::with_capacity(52);
        for syscall in 10_000i64..10_040 {
            let mut clauses = Vec::with_capacity(4);
            for clause_index in 0..4u64 {
                let mut predicates = Vec::with_capacity(4);
                for predicate_index in 0..4u8 {
                    let value = 0x0100_0000_0000_0000u64
                        | ((syscall as u64) << 8)
                        | (clause_index << 4)
                        | u64::from(predicate_index);
                    predicates.push(
                        SeccompArgPredicateV1::new_with_op(
                            predicate_index,
                            u64::MAX,
                            value,
                            SeccompArgPredicateOpV1::MaskedNotEqual,
                        )?,
                    );
                }
                clauses.push(SeccompSyscallClauseV2::new(predicates)?);
            }
            rules.push(SeccompSyscallRuleV2::new_with_clauses(syscall, clauses)?);
        }

        let runtime_syscalls = [
            libc::SYS_read,
            libc::SYS_write,
            libc::SYS_close,
            libc::SYS_mmap,
            libc::SYS_mprotect,
            libc::SYS_munmap,
            libc::SYS_brk,
            libc::SYS_getrandom,
            libc::SYS_prctl,
            libc::SYS_seccomp,
            if include_getppid {
                libc::SYS_getppid
            } else {
                libc::SYS_getuid
            },
            libc::SYS_exit_group,
        ];
        rules.extend(
            runtime_syscalls
                .into_iter()
                .map(|syscall| SeccompSyscallRuleV2::new(syscall, Vec::new()))
                .collect::<Result<Vec<_>, _>>()?,
        );

        SeccompSyscallPolicyV2::new(architecture, rules)
    }

    let cumulative_policy = build_cumulative_policy(architecture, true)
        .unwrap_or_else(|_| unsafe { libc::_exit(160) });
    let cumulative_failure_policy = build_cumulative_policy(architecture, false)
        .unwrap_or_else(|_| unsafe { libc::_exit(161) });
    let cumulative_profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(cumulative_policy.digest())
        .unwrap_or_else(|_| unsafe { libc::_exit(162) });
    let cumulative_failure_profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(cumulative_failure_policy.digest())
        .unwrap_or_else(|_| unsafe { libc::_exit(163) });

    // Pre-resolve the exact raw path used after installation. No libc helper
    // is needed on the irreversible side of the boundary.
    let warmup = c"kernel-dispatch-warmup\n";
    let _ = unsafe {
        libc::syscall(
            libc::SYS_write,
            libc::STDERR_FILENO,
            warmup.as_ptr(),
            warmup.to_bytes().len(),
        )
    };
    let _ = unsafe { *libc::__errno_location() };

    if let Err(error) = install_v2(
        RendererProcessAssignmentId::new(6).unwrap(),
        profile,
        &policy,
    ) {
        eprintln!("maximum-dispatch install_v2 error: {error:?}");
        unsafe { libc::_exit(164) };
    }

    // Eight near-maximum filters remain below MAX_INSNS_PER_PATH. The
    // ninth candidate is intentionally different: it denies getppid(), so an
    // incorrect partial attachment is distinguishable from an ENOMEM refusal.
    for index in 0..8u32 {
        let receipt = install_v2(
            RendererProcessAssignmentId::new(20u128 + u128::from(index)).unwrap(),
            cumulative_profile.clone(),
            &cumulative_policy,
        );
        if receipt.is_err() {
            unsafe { libc::_exit(165) };
        }
    }

    let cumulative_result = install_v2(
        RendererProcessAssignmentId::new(30).unwrap(),
        cumulative_failure_profile,
        &cumulative_failure_policy,
    );
    if !matches!(
        cumulative_result,
        Err(prism_bridge::seccomp::SeccompError::InstallationFailed(errno))
            if errno == libc::ENOMEM
    ) {
        unsafe { libc::_exit(166) };
    }

    // The failed cumulative installation must not attach the distinct
    // candidate filter. All seven prior filters and the base filter allow
    // getppid(); the rejected candidate would deny it.
    let cumulative_probe = unsafe { libc::syscall(libc::SYS_getppid) };
    if cumulative_probe <= 0 {
        unsafe { libc::_exit(167) };
    }

    // write() does not match the first rule's syscall number. Reaching
    // this successful result therefore demonstrates that the kernel followed
    // the large rule's false branch over exactly the maximum generated body.
    let payload = c"kernel-dispatch-ok\n";
    let expected_len = payload.to_bytes().len();
    let allowed = unsafe {
        libc::syscall(
            libc::SYS_write,
            libc::STDERR_FILENO,
            payload.as_ptr(),
            expected_len,
        )
    };
    if allowed != expected_len as i64 {
        unsafe { libc::_exit(160) };
    }

    // A different, unlisted syscall must continue through the unconditional
    // write()/exit_group rules and still reach the compiler's global EPERM
    // terminator. This catches a dispatch offset that lands inside the
    // following rule chain instead of at its exact boundary.
    let denied = unsafe { libc::syscall(libc::SYS_getppid) };
    let denied_errno = unsafe { *libc::__errno_location() };
    if denied != -1 || denied_errno != libc::EPERM {
        unsafe { libc::_exit(161) };
    }

    unsafe { libc::_exit(0) }
}

#[cfg(target_os = "linux")]
#[test]
fn seccomp_maximum_dispatch_offset_is_kernel_enforced() {
    if std::env::var_os("PRISM_SECCOMP_MAX_DISPATCH_CHILD").is_some() {
        maximum_dispatch_offset_child();
    }

    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .arg("--exact")
        .arg("seccomp_maximum_dispatch_offset_is_kernel_enforced")
        .arg("--nocapture")
        .env("PRISM_SECCOMP_MAX_DISPATCH_CHILD", "1")
        .status()
        .expect("failed to launch seccomp maximum-dispatch child");

    assert!(
        status.success(),
        "seccomp maximum-dispatch child failed: {status}"
    );
}

#[cfg(target_os = "linux")]
#[test]
fn seccomp_install_rejects_wrong_architecture_before_enforcement() {
    use prism_bridge::process::{
        RendererProcessAssignmentId, SandboxProfileV1
    };
    use prism_bridge::seccomp::{
        install, SeccompArchitecture, SeccompError, SeccompSyscallPolicyV1
    };

    let current = SeccompArchitecture::current().expect("qualification host architecture");
    let wrong = match current {
        SeccompArchitecture::X86_64 => SeccompArchitecture::Aarch64,
        SeccompArchitecture::Aarch64 => SeccompArchitecture::X86_64,
        SeccompArchitecture::Riscv64 => SeccompArchitecture::X86_64,
    };
    let policy = SeccompSyscallPolicyV1::new(wrong, vec![libc::SYS_getpid])
        .expect("syntactically valid wrong-architecture policy");
    let profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(policy.digest())
        .expect("non-zero policy commitment");

    let result = install(
        RendererProcessAssignmentId::new(3).unwrap(),
        profile,
        &policy,
    );

    assert!(matches!(result, Err(SeccompError::ArchitectureMismatch)));
}


#[cfg(target_os = "linux")]
fn thread_sync_divergent_filter_child() -> ! {
    use prism_bridge::process::{RendererProcessAssignmentId, SandboxProfileV1};
    use prism_bridge::seccomp::{
        install, install_v2, SeccompArgPredicateV1, SeccompArchitecture, SeccompError,
        SeccompSyscallPolicyV1, SeccompSyscallRuleV2, SeccompSyscallPolicyV2,
    };

    #[repr(C)]
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

    const BPF_RET_K: u16 = 0x0006;
    const SECCOMP_SET_MODE_FILTER: libc::c_uint = 1;
    const SECCOMP_RET_ALLOW: u32 = 0x7fff_0000;

    fn read_exact(fd: libc::c_int, bytes: &mut [u8]) -> bool {
        let mut offset = 0usize;
        while offset < bytes.len() {
            let rc = unsafe {
                libc::syscall(
                    libc::SYS_read,
                    fd,
                    bytes[offset..].as_mut_ptr(),
                    bytes.len() - offset,
                )
            };
            if rc <= 0 {
                return false;
            }
            offset += rc as usize;
        }
        true
    }

    fn write_exact(fd: libc::c_int, bytes: &[u8]) -> bool {
        let mut offset = 0usize;
        while offset < bytes.len() {
            let rc = unsafe {
                libc::syscall(
                    libc::SYS_write,
                    fd,
                    bytes[offset..].as_ptr(),
                    bytes.len() - offset,
                )
            };
            if rc <= 0 {
                return false;
            }
            offset += rc as usize;
        }
        true
    }

    let architecture =
        SeccompArchitecture::current().unwrap_or_else(|| unsafe { libc::_exit(180) });

    let mut ready = [-1; 2];
    let mut release = [-1; 2];
    if unsafe { libc::pipe2(ready.as_mut_ptr(), libc::O_CLOEXEC) } != 0
        || unsafe { libc::pipe2(release.as_mut_ptr(), libc::O_CLOEXEC) } != 0
    {
        unsafe { libc::_exit(181) };
    }

    let sibling_ready = ready[1];
    let sibling_release = release[0];

    std::thread::spawn(move || {
        // Attach a deliberately divergent filter tree only to this sibling.
        // An unconditional ALLOW keeps the thread operational while making
        // TSYNC unable to merge the calling thread's filter tree with it.
        if unsafe { libc::prctl(libc::PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) } != 0 {
            unsafe { libc::_exit(182) };
        }
        let filter = SockFilter {
            code: BPF_RET_K,
            jt: 0,
            jf: 0,
            k: SECCOMP_RET_ALLOW,
        };
        let program = SockFprog {
            len: 1,
            filter: &filter,
        };
        let rc = unsafe {
            libc::syscall(
                libc::SYS_seccomp,
                SECCOMP_SET_MODE_FILTER,
                0,
                &program as *const SockFprog,
            )
        };
        if rc != 0 {
            unsafe { libc::_exit(183) };
        }

        let byte = [1u8];
        if !unsafe { write_exact(sibling_ready, &byte) } {
            unsafe { libc::_exit(184) };
        }

        let mut release_byte = [0u8; 1];
        if !unsafe { read_exact(sibling_release, &mut release_byte) } {
            unsafe { libc::_exit(185) };
        }

        // The failed TSYNC must not replace the sibling's divergent filter either.
        // The sibling's unconditional ALLOW therefore keeps getppid() operational.
        let sibling_probe = unsafe { libc::syscall(libc::SYS_getppid) };
        if sibling_probe <= 0 {
            unsafe { libc::_exit(192) };
        }

        let byte = [1u8];
        if !unsafe { write_exact(sibling_ready, &byte) } {
            unsafe { libc::_exit(193) };
        }

        unsafe { libc::_exit(0) }
    });

    let mut ready_byte = [0u8; 1];
    if !unsafe { read_exact(ready[0], &mut ready_byte) } {
        unsafe { libc::_exit(186) };
    }

    let policy = SeccompSyscallPolicyV1::new(
        architecture,
        vec![
            libc::SYS_getpid,
            libc::SYS_read,
            libc::SYS_write,
            libc::SYS_exit_group,
        ],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(187) });
    let profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(policy.digest())
        .unwrap_or_else(|_| unsafe { libc::_exit(188) });

    let result = install(
        RendererProcessAssignmentId::new(7).unwrap(),
        profile,
        &policy,
    );

    // TSYNC_ESRCH must convert divergent-thread synchronization failure into
    // ordinary ESRCH error handling. A positive raw syscall result (thread ID)
    // is itself a failure and must never be treated as successful enforcement.
    if !matches!(
        result,
        Err(SeccompError::InstallationFailed(errno)) if errno == libc::ESRCH
    ) {
        unsafe { libc::_exit(189) };
    }

    // Exercise the same kernel-side TSYNC failure through the parameter-aware
    // V2 installation path. Keeping the divergent sibling in place means both
    // installers must normalize the same underlying failure to ESRCH.
    let v2_rule = SeccompSyscallRuleV2::new(
        libc::SYS_getpid,
        vec![SeccompArgPredicateV1::new(0, 1, 0)
            .unwrap_or_else(|_| unsafe { libc::_exit(195) })],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(196) });
    let v2_policy = SeccompSyscallPolicyV2::new(architecture, vec![v2_rule])
        .unwrap_or_else(|_| unsafe { libc::_exit(197) });
    let v2_profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(v2_policy.digest())
        .unwrap_or_else(|_| unsafe { libc::_exit(198) });

    let v2_result = install_v2(
        RendererProcessAssignmentId::new(9).unwrap(),
        v2_profile,
        &v2_policy,
    );
    if !matches!(
        v2_result,
        Err(SeccompError::InstallationFailed(errno)) if errno == libc::ESRCH
    ) {
        unsafe { libc::_exit(199) };
    }

    // Linux guarantees that a failed TSYNC synchronization does not attach the
    // new filter. Probe the calling thread with getppid(), which is absent from
    // both the V1 and V2 proposed allowlists: success proves no partial filter
    // was installed by either attempted path.
    let caller_probe = unsafe { libc::syscall(libc::SYS_getppid) };
    if caller_probe <= 0 {
        unsafe { libc::_exit(191) };
    }

    let release_byte = [1u8];
    if !unsafe { write_exact(release[1], &release_byte) } {
        unsafe { libc::_exit(190) };
    }

    // Require the sibling's post-failure probe before allowing the child to exit.
    let mut sibling_verified = [0u8; 1];
    if !unsafe { read_exact(ready[0], &mut sibling_verified) } {
        unsafe { libc::_exit(194) };
    }

    unsafe { libc::_exit(0) }
}

#[cfg(target_os = "linux")]
fn thread_sync_strict_mode_child() -> ! {
    use prism_bridge::process::{RendererProcessAssignmentId, SandboxProfileV1};
    use prism_bridge::seccomp::{
        install, install_v2, SeccompArchitecture, SeccompArgPredicateV1, SeccompError,
        SeccompSyscallPolicyV1, SeccompSyscallPolicyV2, SeccompSyscallRuleV2,
    };

    const SECCOMP_SET_MODE_STRICT: libc::c_uint = 0;

    fn strict_exit(code: libc::c_int) -> ! {
        unsafe {
            libc::syscall(libc::SYS_exit, code);
        }
        std::process::abort();
    }

    fn read_exact(fd: libc::c_int, bytes: &mut [u8]) -> bool {
        let mut offset = 0usize;
        while offset < bytes.len() {
            let rc = unsafe {
                libc::syscall(
                    libc::SYS_read,
                    fd,
                    bytes[offset..].as_mut_ptr(),
                    bytes.len() - offset,
                )
            };
            if rc <= 0 {
                return false;
            }
            offset += rc as usize;
        }
        true
    }

    fn write_exact(fd: libc::c_int, bytes: &[u8]) -> bool {
        let mut offset = 0usize;
        while offset < bytes.len() {
            let rc = unsafe {
                libc::syscall(
                    libc::SYS_write,
                    fd,
                    bytes[offset..].as_ptr(),
                    bytes.len() - offset,
                )
            };
            if rc <= 0 {
                return false;
            }
            offset += rc as usize;
        }
        true
    }

    let architecture =
        SeccompArchitecture::current().unwrap_or_else(|| unsafe { libc::_exit(200) });

    let mut ready = [-1; 2];
    let mut release = [-1; 2];
    if unsafe { libc::pipe2(ready.as_mut_ptr(), libc::O_CLOEXEC) } != 0
        || unsafe { libc::pipe2(release.as_mut_ptr(), libc::O_CLOEXEC) } != 0
    {
        unsafe { libc::_exit(201) };
    }

    let sibling_ready = ready[1];
    let sibling_release = release[0];

    std::thread::spawn(move || {
        // Strict mode is a distinct kernel reason for TSYNC refusal from a
        // divergent filter tree. After entry, only raw read/write/exit are
        // used, which are the operations permitted by SECCOMP_MODE_STRICT.
        let rc = unsafe {
            libc::syscall(
                libc::SYS_seccomp,
                SECCOMP_SET_MODE_STRICT,
                0,
                std::ptr::null::<libc::c_void>(),
            )
        };
        if rc != 0 {
            unsafe { libc::_exit(202) };
        }

        let byte = [1u8];
        if !unsafe { write_exact(sibling_ready, &byte) } {
            strict_exit(203);
        }

        let mut release_byte = [0u8; 1];
        if !unsafe { read_exact(sibling_release, &mut release_byte) } {
            strict_exit(204);
        }

        strict_exit(0)
    });

    let mut ready_byte = [0u8; 1];
    if !unsafe { read_exact(ready[0], &mut ready_byte) } {
        unsafe { libc::_exit(205) };
    }

    let policy = SeccompSyscallPolicyV1::new(
        architecture,
        vec![
            libc::SYS_getpid,
            libc::SYS_read,
            libc::SYS_write,
            libc::SYS_exit_group,
        ],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(206) });
    let profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(policy.digest())
        .unwrap_or_else(|_| unsafe { libc::_exit(207) });

    let result = install(
        RendererProcessAssignmentId::new(8).unwrap(),
        profile,
        &policy,
    );

    if !matches!(
        result,
        Err(SeccompError::InstallationFailed(errno)) if errno == libc::ESRCH
    ) {
        unsafe { libc::_exit(208) };
    }

    // Exercise the same strict-mode TSYNC refusal through the parameter-aware
    // V2 installation path. The sibling remains in SECCOMP_MODE_STRICT, so the
    // kernel must reject synchronization before attaching either new filter.
    let v2_rule = SeccompSyscallRuleV2::new(
        libc::SYS_getpid,
        vec![SeccompArgPredicateV1::new(0, 1, 0)
            .unwrap_or_else(|_| unsafe { libc::_exit(211) })],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(212) });
    let v2_policy = SeccompSyscallPolicyV2::new(architecture, vec![v2_rule])
        .unwrap_or_else(|_| unsafe { libc::_exit(213) });
    let v2_profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(v2_policy.digest())
        .unwrap_or_else(|_| unsafe { libc::_exit(214) });

    let v2_result = install_v2(
        RendererProcessAssignmentId::new(10).unwrap(),
        v2_profile,
        &v2_policy,
    );
    if !matches!(
        v2_result,
        Err(SeccompError::InstallationFailed(errno)) if errno == libc::ESRCH
    ) {
        unsafe { libc::_exit(215) };
    }

    // A failed TSYNC must not attach or replace any seccomp filter on
    // either thread. The caller therefore remains unrestricted and can execute
    // getppid(), while the strict sibling remains governed by strict mode.
    let caller_probe = unsafe { libc::syscall(libc::SYS_getppid) };
    if caller_probe <= 0 {
        unsafe { libc::_exit(209) };
    }

    let release_byte = [1u8];
    if !unsafe { write_exact(release[1], &release_byte) } {
        unsafe { libc::_exit(210) };
    }

    unsafe { libc::_exit(0) }
}

#[cfg(target_os = "linux")]
#[test]
fn seccomp_tsync_esrch_normalizes_divergent_filter_failure() {
    if std::env::var_os("PRISM_SECCOMP_DIVERGENT_TSYNC_CHILD").is_some() {
        thread_sync_divergent_filter_child();
    }

    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .arg("--exact")
        .arg("seccomp_tsync_esrch_normalizes_divergent_filter_failure")
        .arg("--nocapture")
        .env("RUST_TEST_THREADS", "1")
        .env("PRISM_SECCOMP_DIVERGENT_TSYNC_CHILD", "1")
        .status()
        .expect("failed to launch divergent-filter TSYNC child");

    assert!(
        status.success(),
        "divergent-filter TSYNC child failed: {status}"
    );
}

#[cfg(target_os = "linux")]
fn thread_sync_child() -> ! {
    use prism_bridge::process::{RendererProcessAssignmentId, SandboxProfileV1};
    use prism_bridge::seccomp::{install, SeccompArchitecture, SeccompSyscallPolicyV1};

    #[repr(C)]
    #[derive(Clone, Copy)]
    struct Observation {
        result: i64,
        errno: i32,
    }

    fn read_exact(fd: libc::c_int, bytes: &mut [u8]) -> bool {
        let mut offset = 0usize;
        while offset < bytes.len() {
            let rc = unsafe {
                libc::syscall(
                    libc::SYS_read,
                    fd,
                    bytes[offset..].as_mut_ptr(),
                    bytes.len() - offset,
                )
            };
            if rc <= 0 {
                return false;
            }
            offset += rc as usize;
        }
        true
    }

    fn write_exact(fd: libc::c_int, bytes: &[u8]) -> bool {
        let mut offset = 0usize;
        while offset < bytes.len() {
            let rc = unsafe {
                libc::syscall(
                    libc::SYS_write,
                    fd,
                    bytes[offset..].as_ptr(),
                    bytes.len() - offset,
                )
            };
            if rc <= 0 {
                return false;
            }
            offset += rc as usize;
        }
        true
    }

    let architecture =
        SeccompArchitecture::current().unwrap_or_else(|| unsafe { libc::_exit(100) });
    let policy = SeccompSyscallPolicyV1::new(
        architecture,
        vec![
            libc::SYS_getpid,
            libc::SYS_read,
            libc::SYS_write,
            libc::SYS_exit_group,
        ],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(101) });
    let profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(policy.digest())
        .unwrap_or_else(|_| unsafe { libc::_exit(102) });

    // Use pipes for deterministic pre/post-install rendezvous. The test policy
    // explicitly permits read/write, so the synchronization channel remains
    // usable after seccomp installation without introducing sched_yield/futex
    // dependencies that would otherwise complicate the enforcement probe.
    let mut ready = [-1; 2];
    let mut release = [-1; 2];
    let mut observed = [-1; 2];
    if unsafe { libc::pipe2(ready.as_mut_ptr(), libc::O_CLOEXEC) } != 0
        || unsafe { libc::pipe2(release.as_mut_ptr(), libc::O_CLOEXEC) } != 0
        || unsafe { libc::pipe2(observed.as_mut_ptr(), libc::O_CLOEXEC) } != 0
    {
        unsafe { libc::_exit(103) };
    }

    let thread_ready = ready[1];
    let thread_release = release[0];
    let thread_observed = observed[1];

    std::thread::spawn(move || {
        // Pre-resolve the exact raw syscall/errno paths used for the
        // post-install probe on this thread. This prevents lazy PLT/TLS
        // initialization from turning a correct TSYNC denial into an unrelated
        // process crash.
        let _ = unsafe { libc::syscall(libc::SYS_getppid) };
        let _ = unsafe { *libc::__errno_location() };

        let ready_byte = [1u8];
        if !unsafe { write_exact(thread_ready, &ready_byte) } {
            unsafe { libc::_exit(104) };
        }

        let mut release_byte = [0u8; 1];
        if !unsafe { read_exact(thread_release, &mut release_byte) } {
            unsafe { libc::_exit(105) };
        }

        // Use only raw libc/kernel paths after installation. The read/write
        // channel is explicitly allowlisted solely to make this TSYNC probe
        // deterministic across native runner architectures.
        let result = unsafe { libc::syscall(libc::SYS_getppid) };
        let errno = unsafe { *libc::__errno_location() };
        let observation = Observation {
            result: result as i64,
            errno,
        };
        let bytes = unsafe {
            std::slice::from_raw_parts(
                (&observation as *const Observation).cast::<u8>(),
                std::mem::size_of::<Observation>(),
            )
        };
        if !unsafe { write_exact(thread_observed, bytes) } {
            unsafe { libc::_exit(106) };
        }

        loop {
            std::hint::spin_loop();
        }
    });

    // Both threads have reached the pre-install rendezvous. Because the
    // synchronization primitive itself remains available after installation,
    // the test no longer depends on a fixed spin-loop budget for scheduling.
    let mut ready_byte = [0u8; 1];
    if !unsafe { read_exact(ready[0], &mut ready_byte) } {
        unsafe { libc::_exit(107) };
    }

    if install(
        RendererProcessAssignmentId::new(2).unwrap(),
        profile,
        &policy,
    )
    .is_err()
    {
        unsafe { libc::_exit(108) };
    }

    let release_byte = [1u8];
    if !unsafe { write_exact(release[1], &release_byte) } {
        unsafe { libc::_exit(109) };
    }

    let mut bytes = [0u8; std::mem::size_of::<Observation>()];
    if !unsafe { read_exact(observed[0], &mut bytes) } {
        unsafe { libc::_exit(110) };
    }

    let observation = unsafe {
        std::ptr::read_unaligned(bytes.as_ptr().cast::<Observation>())
    };

    // getppid() always returns a positive parent PID when it executes. -1
    // therefore proves that the sibling thread was filtered and received the
    // policy's default EPERM action rather than merely surviving the TSYNC
    // installation.
    if observation.result != -1 || observation.errno != libc::EPERM {
        unsafe { libc::_exit(111) };
    }

    unsafe { libc::_exit(0) }
}

#[cfg(target_os = "linux")]
#[test]
fn seccomp_tsync_esrch_normalizes_strict_mode_failure() {
    if std::env::var_os("PRISM_SECCOMP_STRICT_TSYNC_CHILD").is_some() {
        thread_sync_strict_mode_child();
    }

    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .arg("--exact")
        .arg("seccomp_tsync_esrch_normalizes_strict_mode_failure")
        .arg("--nocapture")
        .env("RUST_TEST_THREADS", "1")
        .env("PRISM_SECCOMP_STRICT_TSYNC_CHILD", "1")
        .status()
        .expect("failed to launch strict-mode TSYNC child");

    assert!(
        status.success(),
        "strict-mode TSYNC child failed: {status}"
    );
}

#[cfg(target_os = "linux")]
#[test]
fn seccomp_tsync_constrains_existing_sibling_threads() {
    if std::env::var_os("PRISM_SECCOMP_THREAD_CHILD").is_some() {
        thread_sync_child();
    }

    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .arg("--exact")
        .arg("seccomp_tsync_constrains_existing_sibling_threads")
        .arg("--nocapture")
        .env("RUST_TEST_THREADS", "1")
        .env("PRISM_SECCOMP_THREAD_CHILD", "1")
        .status()
        .expect("failed to launch seccomp thread-sync child");

    assert!(status.success(), "seccomp thread-sync child failed: {status}");
}
