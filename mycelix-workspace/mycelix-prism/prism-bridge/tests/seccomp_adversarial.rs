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

    if install(
        RendererProcessAssignmentId::new(1).unwrap(),
        SandboxProfileV1::renderer_default(),
        &policy,
    )
    .is_err()
    {
        unsafe { libc::_exit(92) };
    }

    if unsafe { libc::getpid() } <= 0 {
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
fn thread_sync_child() -> ! {
    use prism_bridge::process::{RendererProcessAssignmentId, SandboxProfileV1};
    use prism_bridge::seccomp::{install, SeccompArchitecture, SeccompSyscallPolicyV1};
    use std::sync::atomic::{AtomicBool, AtomicI64, Ordering};
    use std::sync::Arc;

    let architecture = SeccompArchitecture::current().unwrap_or_else(|| unsafe { libc::_exit(100) });
    let policy = SeccompSyscallPolicyV1::new(
        architecture,
        vec![libc::SYS_getpid, libc::SYS_write, libc::SYS_exit_group],
    )
    .unwrap_or_else(|_| unsafe { libc::_exit(101) });

    // Spawn the sibling before installing the filter. After installation both
    // threads must be constrained by the same filter tree.
    let release = Arc::new(AtomicBool::new(false));
    let observed = Arc::new(AtomicI64::new(i64::MIN));
    let observed_errno = Arc::new(std::sync::atomic::AtomicI32::new(i32::MIN));
    let thread_release = Arc::clone(&release);
    let thread_observed = Arc::clone(&observed);
    let thread_errno = Arc::clone(&observed_errno);

    std::thread::spawn(move || {
        while !thread_release.load(Ordering::Acquire) {
            std::hint::spin_loop();
        }
        // Use the raw syscall here too: the enforcement assertion must cross
        // the libc boundary and observe the kernel's actual errno result.
        let result = unsafe { libc::syscall(libc::SYS_getppid) };
        let errno = std::io::Error::last_os_error()
            .raw_os_error()
            .unwrap_or_default();
        thread_observed.store(i64::from(result), Ordering::Release);
        thread_errno.store(errno, Ordering::Release);
        loop {
            std::hint::spin_loop();
        }
    });

    if install(
        RendererProcessAssignmentId::new(2).unwrap(),
        SandboxProfileV1::renderer_default(),
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
