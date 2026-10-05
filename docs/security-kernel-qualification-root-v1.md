# Security Kernel Independent Qualification Root v1

## Purpose

The ordinary Security Kernel Qualification workflow is a minimal fast PR trigger carrier.
GitHub resolves `pull_request` workflows from the event-associated commit, while
`workflow_run` listeners must exist on the default branch. Therefore the carrier is
deliberately untrusted and contains only a pinned checkout with read-only repository
permission; it is a trigger mechanism, not a security decision root.

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
candidate SHA, and exact open PR before invoking S1. The PR identity is taken from the
verified source run's `pull_requests` relation and then re-fetched by PR number, so the
binding does not depend on the base repository's commit lookup and remains valid for
fork-origin PR heads.

The dispatcher binds the source workflow name `Security Kernel Qualification` and path
`.github/workflows/security-kernel-qualification.yml`. The source workflow is only a
non-authoritative trigger carrier: its code and outcome are not trusted, and S0 requires
the run to be a completed `pull_request` run from that exact path/name. S0 then checks
that the S1 reusable-workflow blob at S0's own executing commit matches the registered
S1 profile. S1 is invoked with `./.github/workflows/security-kernel-independent-qualification.yml`,
so GitHub resolves the called workflow from the same commit as the caller rather than
from a separate mutable branch/tag. The S0 caller exposes only read permissions to the
called workflow and does not provide a manual or API dispatch entrypoint.

## S1 — trusted exact-head executor

`security-kernel-independent-qualification.yml` is a `workflow_call`-only reusable
workflow invoked locally by S0. GitHub documents that a same-repository local reusable
workflow is taken from the same commit as its caller. The candidate does not supply the
workflow definition or harness commands.

S1:

- validates that the source run is the exact completed `pull_request` carrier run and binds its candidate PR/repository/SHA;
- proves its caller is the trusted S0 workflow and that the called S1 blob at that caller
  commit matches the registered S1 profile;
- binds the candidate PR number to the verified source run's `pull_requests` relation
  before re-reading the current PR object;
- fetches the exact candidate repository and commit SHA rather than a mutable PR branch;
- materializes the exact commit as source data;
- executes the trusted qualification workflow from the exact caller commit rather than mutable `main`;
- performs an independent static trust-surface audit;
- runs rustfmt, default-feature tests, identity-feature tests, and Clippy using Rust 1.99.0;
- captures the exact candidate source digest as a trusted step output before dependency acquisition or candidate execution, using unambiguous length-framed path/content records, and records the candidate tree, lockfile, vendor closure, sandbox image, and trusted workflow identities; and
- fails if any committed candidate source changes during qualification; build output is placed outside the source mount in a dedicated `/target` tmpfs.

The candidate executes without repository write permission, secrets, or OIDC access. The S0 caller explicitly sets `cache-mode: none`, so the reusable S1 execution does not receive persistent Actions cache access. S1 also requires the source PR to remain an exact open-head match at execution time, records the pre-execution source digest in a trusted workflow step output, and requires a committed Security Kernel `Cargo.lock` while rejecting candidate-controlled Cargo configuration at the relevant workspace/config hierarchy. The lockfile policy also rejects non-crates.io package sources, preventing a candidate from turning dependency acquisition into arbitrary Git/custom-registry network access. The lockfile digest is captured before candidate execution. Candidate dependency acquisition uses only the committed manifest/lockfile in a constrained non-root container; candidate source and code are not present during that networked fetch phase. The resulting vendor tree is then mounted read-only for candidate execution. Candidate Cargo gates use the Rust 1.99.0 compiler contained in the immutable sandbox image, `--frozen`, `network=none` / offline networking, a read-only source tree, and fresh disposable containers with dropped capabilities, `no-new-privileges`, resource bounds, private PID/IPC namespaces, and no Docker socket. These controls substantially reduce candidate access to the host runner, but the Docker daemon and host kernel remain trusted infrastructure. Stronger runtime isolation is tracked in #4152.

## S2 — trusted result verifier

`security-kernel-trusted-result-verifier.yml` runs from the default branch after S1.
It does not execute candidate code and does not consume candidate-produced PASS text.
It independently validates:

- S0 workflow path and `workflow_run` event;
- the source carrier path/name and `pull_request` event are exact;
- the exact S0 workflow blob executed by the completed run matches the registered dispatcher profile;
- the exact S1 reusable-workflow blob at that same S0 commit matches the registered S1 profile;
- the S0 workflow commit is an ancestor of current protected `main`;
- exact candidate SHA and current PR head binding;
- candidate PR identity derived from the source run relation rather than a base-repository
  commit search;
- exact source qualification run and attempt;
- the complete required S1 gate set for the exact workflow run attempt and their individual successful conclusions, with the S0 reusable-workflow call restricted to `cache-mode: none`.

The current S2 implementation is deliberately read-only. It produces a machine-readable
verification result in the trusted job log but does not hold status-write permission. The
verifier reads the S1 workflow blob at the exact workflow commit recorded by the run,
not merely at current `main`, preventing a different later workflow version from being
accepted because it replaced the exact execution commit. It also rejects PR-head drift
between source qualification and result verification.
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
containers consume read-only. The vendor digest uses the same unambiguous length-framed
record encoding for its relative paths and file contents.

The candidate phase uses the immutable Rust image
`docker.io/library/rust@sha256:9af5f5f37d3035dd18d216e348e946ff1fc8c7fa7998c7443cedfe880231110d`.
The Docker image is selected directly by the verified amd64 manifest digest rather than a mutable tag. The upstream official image metadata records this exact manifest as `sha256:9af5f5f37d3035dd18d216e348e946ff1fc8c7fa7998c7443cedfe880231110d` under the `rust:1.99.0-slim-bookworm` image.
The current S1 run requests `linux/amd64` explicitly.

Each candidate gate gets a fresh disposable container. The trusted workflow checkout is
bound to the exact S0 caller commit; candidate source and vendor contents are separate
read-only mounts, while writable state is confined to bounded tmpfs locations and an
isolated build target. Network access is disabled for candidate execution.
The candidate receives no usable GitHub token, OIDC request token, runtime token, or Docker socket.

The qualification profile intentionally does not claim that these controls provide a kernel
or container-runtime security theorem. The host remains trusted infrastructure, and the
profile's security claim is limited to the registered controls being observed by the trusted
harness and independently checked by S2.
