//! Hosted adversarial seccomp checks.
//!
//! The filter is irreversible for the calling process, so the enforcement test
//! executes the installation in a dedicated child. The parent remains
//! unsandboxed and can report the result to Cargo's test runner.

#[cfg(target_os = "linux")]
fn child() -> ! {
    use prism_bridge::process::{RendererProcessAssignmentId, SandboxProfileV1};
    use prism_bridge::seccomp::{install, SeccompArchitecture, SeccompSyscallPolicyV1};

    let architecture = SeccompArchitecture::current().unwrap_or_else(|| unsafe {
        libc::_exit(90)
    });

    // Keep the post-install workload deliberately tiny: the test proves that
    // an allowlisted syscall reaches the kernel and that a non-allowlisted
    // syscall receives the adapter's default EPERM action. The production
    // renderer policy must be derived separately from qualified workload
    // evidence.
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

    // getpid is explicitly allowlisted and should still execute normally.
    if unsafe { libc::getpid() } <= 0 {
        unsafe { libc::_exit(93) };
    }

    // getppid is deliberately absent from the policy and therefore must be
    // denied with the adapter's default EPERM action.
    let denied = unsafe { libc::getppid() };
    let errno = std::io::Error::last_os_error().raw_os_error();
    if denied != -1 || errno != Some(libc::EPERM) {
        unsafe { libc::_exit(94) };
    }

    unsafe { libc::_exit(0) }
}

#[cfg(target_os = "linux")]
fn main() {
    if std::env::var_os("PRISM_SECCOMP_CHILD").is_some() {
        child();
    }

    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .env("PRISM_SECCOMP_CHILD", "1")
        .status()
        .expect("failed to launch seccomp child");
    assert!(status.success(), "seccomp child failed: {status}");
}

#[cfg(not(target_os = "linux"))]
fn main() {
    // The adapter is Linux-specific; non-Linux CI must remain green without
    // pretending that Linux enforcement was exercised.
}
