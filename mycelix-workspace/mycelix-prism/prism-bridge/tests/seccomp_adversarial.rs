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

    let denied = unsafe { libc::getppid() };
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
