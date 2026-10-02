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

    // Spawn the sibling before installing the filter. After installation both
    // threads must be constrained by the same filter tree.
    let release = Arc::new(AtomicBool::new(false));
    let observed = Arc::new(AtomicI64::new(i64::MIN));
    let observed_errno = Arc::new(std::sync::atomic::AtomicI32::new(i32::MIN));
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

    for _ in 0..10_000_000 {
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
        .env("PRISM_SECCOMP_THREAD_CHILD", "1")
        .status()
        .expect("failed to launch seccomp thread-sync child");

    assert!(status.success(), "seccomp thread-sync child failed: {status}");
}
