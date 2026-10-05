# Security Kernel Independent Qualification Root v1

## Purpose

The ordinary Security Kernel Qualification workflow remains the fast PR feedback lane.
It is intentionally unprivileged, but GitHub documents that a pull_request workflow uses
the workflow definition associated with the pull request's merge commit. Therefore the
PR workflow cannot be treated as an independent trust root for deciding whether its own
security workflow is correct.

This profile separates the planes:

```text
PR qualification
  -> trusted default-branch dispatcher (S0)
  -> trusted exact-head executor (S1)
  -> trusted read-only result verifier (S2)
```

## S0 — trusted dispatcher

`security-kernel-trusted-dispatch.yml` runs from the default branch on completion of the
existing Security Kernel Qualification workflow. It never checks out candidate code.
It validates the source workflow ID/path, exact source run/attempt, candidate repository,
candidate SHA, and exact open PR before dispatching S1.

The dispatcher binds source workflow ID `372951439` and path
`.github/workflows/security-kernel-qualification.yml`. Changing either is a trusted
configuration change and must therefore pass ordinary protected-branch review.

## S1 — trusted exact-head executor

`security-kernel-independent-qualification.yml` exists on the default branch and uses
`workflow_dispatch`. The candidate does not supply the workflow definition or harness
commands.

S1:

- validates that its inputs came from an exact completed source qualification run;
- fetches the exact candidate repository and commit SHA rather than a mutable PR branch;
- materializes the exact commit as source data;
- performs an independent static trust-surface audit;
- runs rustfmt, default-feature tests, identity-feature tests, and Clippy using Rust 1.99.0;
- records the candidate tree, lockfile, vendor closure, sandbox image, and trusted workflow identities; and
- fails if any committed candidate source changes during qualification; build output is placed outside the source mount in a dedicated `/target` tmpfs.

The candidate executes without repository write permission, secrets, or OIDC access. S1 also requires the source PR to remain an exact open-head match at execution time, records the pre-execution source digest in a trusted workflow step output, and requires a committed Security Kernel `Cargo.lock` while rejecting candidate-controlled Cargo configuration at the relevant workspace/config hierarchy. The lockfile policy also rejects non-crates.io package sources, preventing a candidate from turning dependency acquisition into arbitrary Git/custom-registry network access. The lockfile digest is captured before candidate execution. Candidate dependency acquisition uses only the committed manifest/lockfile in a constrained non-root container; candidate source and code are not present during that networked fetch phase. The resulting vendor tree is then mounted read-only for candidate execution. Candidate Cargo gates use Rust 1.99.0 from a read-only mounted sysroot, `--frozen`, `network=none` / offline networking, a read-only source tree, and fresh disposable containers with dropped capabilities, `no-new-privileges`, resource bounds, private PID/IPC namespaces, and no Docker socket. These controls substantially reduce candidate access to the host runner, but this remains a defense-in-depth container boundary rather than a proof of Linux kernel/Docker-daemon escape resistance.

## S2 — trusted result verifier

`security-kernel-trusted-result-verifier.yml` runs from the default branch after S1.
It does not execute candidate code and does not consume candidate-produced PASS text.
It independently validates:

- S1 workflow path and `workflow_dispatch` event;
- S1 workflow reference is the default branch;
- the exact workflow blob executed by the S1 run matches the registered immutable profile;
- S1 workflow commit is an ancestor of current protected `main`;
- exact candidate SHA and current PR head binding;
- exact source qualification run and attempt;
- the complete required S1 gate set and their individual successful conclusions.

The current S2 implementation is deliberately read-only. It produces a machine-readable
verification result in the trusted job log but does not hold status-write permission. The
verifier reads the S1 workflow blob at the exact workflow commit recorded by the run,
not merely at current `main`, preventing a malicious workflow version from being accepted
because it was later reverted. It also rejects PR-head drift between S1 dispatch and result
verification.
This keeps result publication as a separate least-privilege decision rather than silently
granting another trusted workflow mutation authority.

## Evidence ceiling

PASS under this profile means that the selected gates passed under the registered trusted
qualification mechanism for one exact immutable candidate commit.

PASS does not mean:

- formal verification;
- absence of implementation vulnerabilities;
- runner or kernel escape resistance;
- GitHub platform compromise resistance;
- independent human security review; or
- runtime authorization.

A qualification receipt is evidence, never runtime authority.

## S1 dependency and sandbox boundary

The dependency phase and candidate-code phase are intentionally different trust zones.

The dependency phase is allowed outbound network access only from a disposable non-root
container and receives the exact committed bridge manifest and lockfile, without candidate
source, repository credentials, GitHub/OIDC tokens, Docker socket, or privileged capabilities.
It emits a vendor tree and generated Cargo source configuration that the later candidate
containers consume read-only.

The candidate phase uses the immutable Rust image
`docker.io/library/rust@sha256:9af5f5f37d3035dd18d216e348e946ff1fc8c7fa7998c7443cedfe880231110d`.
The Docker image is selected directly by the verified amd64 manifest digest rather than a mutable tag. The upstream official image metadata records this exact manifest as `sha256:9af5f5f37d3035dd18d216e348e946ff1fc8c7fa7998c7443cedfe880231110d` under the `rust:1.99.0-slim-bookworm` image.
The current S1 run requests `linux/amd64` explicitly.

Each candidate gate gets a fresh disposable container. Candidate source, vendor contents,
and the Rust sysroot are read-only mounts. Writable state is confined to bounded tmpfs
locations and an isolated build target. Network access is disabled for candidate execution.
The candidate receives no GitHub token, OIDC request token, runtime token, or Docker socket.

The qualification profile intentionally does not claim that these controls provide a kernel
or container-runtime security theorem. The host remains trusted infrastructure, and the
profile's security claim is limited to the registered controls being observed by the trusted
harness and independently checked by S2.
