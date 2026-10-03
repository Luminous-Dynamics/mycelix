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

    let unix = SeccompSyscallClauseV2::new(vec![
        SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_UNIX as u64)
            .unwrap_or_else(|_| unsafe { libc::_exit(131) }),
    ])
    .unwrap_or_else(|_| unsafe { libc::_exit(132) });
    let netlink = SeccompSyscallClauseV2::new(vec![
        SeccompArgPredicateV1::new(0, u64::MAX, libc::AF_NETLINK as u64)
            .unwrap_or_else(|_| unsafe { libc::_exit(133) }),
    ])
    .unwrap_or_else(|_| unsafe { libc::_exit(134) });
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
        unsafe { libc::_exit(142) };
    }

    disjunctive_stage(b"H-inet-denied-ok\n");
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
    // compiler's maximum V2 rule-body shape: 4 * (4 * 7 + 1) = 116
    // instructions. Make read() the first rule and write() the following
    // unconditional rule. read < write < exit_group on all three supported
    // Linux architectures, so the write() probe must take a 116-instruction
    // forward jump. The final unconditional exit_group rule keeps the child
    // termination path explicit and fail-closed.
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

    let policy = SeccompSyscallPolicyV2::new(
        architecture,
        vec![large_rule, unconditional_allowed, unconditional_exit],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(157) });
    let profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(policy.digest())
        .unwrap_or_else(|_| unsafe { libc::_exit(158) });

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
        unsafe { libc::_exit(159) };
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
fn thread_sync_child() -> ! {
    use prism_bridge::process::{RendererProcessAssignmentId, SandboxProfileV1};
    use prism_bridge::seccomp::{install, SeccompArchitecture, SeccompSyscallPolicyV1};
    use std::sync::atomic::{AtomicBool, AtomicI64, Ordering};
    use std::sync::Arc;

    let architecture =
        SeccompArchitecture::current().unwrap_or_else(|| unsafe { libc::_exit(100) });
    let policy = SeccompSyscallPolicyV1::new(
        architecture,
        vec![libc::SYS_getpid, libc::SYS_write, libc::SYS_exit_group],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(101) });
    let profile = SandboxProfileV1::renderer_default()
        .with_syscall_policy_digest(policy.digest())
        .unwrap_or_else(|_| unsafe { libc::_exit(102) });

    // Establish a pre-install rendezvous so the sibling is definitely alive
    // and executing before TSYNC is attempted. This keeps scheduling latency
    // after installation from being mistaken for a synchronization failure.
    let ready = Arc::new(std::sync::Barrier::new(2));
    let release = Arc::new(AtomicBool::new(false));
    let observed = Arc::new(AtomicI64::new(i64::MIN));
    let observed_errno = Arc::new(std::sync::atomic::AtomicI32::new(i32::MIN));
    let thread_ready = Arc::clone(&ready);
    let thread_release = Arc::clone(&release);
    let thread_observed = Arc::clone(&observed);
    let thread_errno = Arc::clone(&observed_errno);

    std::thread::spawn(move || {
        // Pre-resolve the exact raw syscall/errno paths used for the
        // post-install probe on this thread. This prevents lazy PLT/TLS
        // initialization from turning a correct TSYNC denial into an unrelated
        // process crash.
        let _ = unsafe { libc::syscall(libc::SYS_getppid) };
        let _ = unsafe { *libc::__errno_location() };
        thread_ready.wait();

        while !thread_release.load(Ordering::Acquire) {
            std::hint::spin_loop();
        }
        // Use only raw libc/kernel paths after installation. Avoid Rust's
        // higher-level errno helpers here because they can introduce unrelated
        // runtime work into a deliberately tiny, fail-closed seccomp probe.
        let result = unsafe { libc::syscall(libc::SYS_getppid) };
        let errno = unsafe { *libc::__errno_location() };
        thread_observed.store(i64::from(result), Ordering::Release);
        thread_errno.store(errno, Ordering::Release);
        loop {
            std::hint::spin_loop();
        }
    });

    // Both threads have reached the pre-install rendezvous.
    ready.wait();

    if install(
        RendererProcessAssignmentId::new(2).unwrap(),
        profile,
        &policy,
    )
    .is_err()
    {
        unsafe { libc::_exit(102) };
    }

    release.store(true, Ordering::Release);

    for _ in 0..100_000_000 {
        if observed.load(Ordering::Acquire) != i64::MIN {
            break;
        }
        std::hint::spin_loop();
    }

    // getppid() always returns a positive parent PID when it executes. -1
    // therefore proves that the sibling thread was filtered and received the
    // policy's default EPERM action rather than merely surviving the TSYNC
    // installation.
    if observed.load(Ordering::Acquire) != -1
        || observed_errno.load(Ordering::Acquire) != libc::EPERM
    {
        unsafe { libc::_exit(103) };
    }

    unsafe { libc::_exit(0) }
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
