# Security Kernel Independent Qualification Root v1

## Purpose

The ordinary Security Kernel Qualification workflow is an untrusted candidate feedback
lane. Its workflow definition and its code/configuration are controlled by the candidate PR
and therefore are not a security trust root.

The trusted qualification path is:

```text
candidate PR
  -> trusted default-branch S0 (`pull_request_target`, metadata only)
  -> same-commit local reusable S1 (`workflow_call`, hostile candidate execution)
  -> trusted default-branch S2 (`workflow_run`, read-only verification)
```

S0 is the authority root for discovering the current PR subject. S1 executes the exact
candidate commit under the registered isolation profile. S2 verifies the exact completed
S0/S1 attempt without executing candidate code or trusting candidate-produced PASS text.

## S0 — trusted PR discovery and dispatch

`.github/workflows/security-kernel-trusted-dispatch.yml` is intended to live on protected
default `main` and uses `pull_request_target`. GitHub documents that this event executes
the workflow from the base repository's default branch rather than the pull request head,
which prevents a candidate from disabling the trusted dispatcher by editing its own PR
workflow.

S0 is deliberately metadata-only:

- no checkout of candidate source;
- no candidate artifact download or execution;
- no secrets referenced;
- no candidate-controlled source carrier is required;
- `actions: read`, `contents: read`, and `pull-requests: read`;
- no workflow-dispatch API call;
- current PR identity is taken from the event and then re-read from the GitHub PR API;
- the candidate repository, exact head SHA, base repository, and base branch are bound before
  invoking S1;
- S1 is invoked as a same-repository local reusable workflow:
  `./.github/workflows/security-kernel-independent-qualification.yml`;
- the reusable call explicitly sets `cache-mode: none`.

The local reusable-workflow form is important: GitHub resolves a same-repository local
reusable workflow from the same commit as the caller. Therefore S0 and S1 share one
immutable workflow commit rather than relying on a mutable branch/tag dispatch.

### Public-repository policy prerequisite

GitHub's current public-repository default Actions event policy blocks
`pull_request_target` and is scheduled for enforcement on **November 2, 2026** unless
the repository has an applicable event policy that explicitly permits it.
This requirement is tracked in **#4195**.

S0 must remain metadata-only when that exception is installed. GitHub specifically warns
against checking out, building, or executing pull-request code from `pull_request_target`
with privileged access.

## S1 — exact-head hostile-code executor

`.github/workflows/security-kernel-independent-qualification.yml` is
`workflow_call`-only and has no manual/API dispatch entrypoint.

S1 first proves:

- `github.event_name == pull_request_target`;
- `github.workflow_ref` is exactly the trusted S0 workflow on `refs/heads/main`;
- `github.workflow_sha` is a valid commit SHA for the S0 caller;
- the called S1 workflow blob at that caller commit matches the registered S1 profile;
- the candidate PR number/repository/head SHA passed by S0 exactly match the current PR.

S1 resolves the exact candidate commit and tree through the GitHub API and enforces the
resource profile before any candidate blob contents are materialized. Hostile Git transport,
object validation, and archive extraction then occur only inside a dedicated pinned
non-root networked fetch sandbox. No candidate repository `.git` data is fetched, parsed,
or archived by Git on the trusted host runner.

Git's own security guidance notes that the fetch/upload-pack attack surface is substantial,
so the fetch sandbox is a separate trust zone rather than part of the trusted host phase.
citeturn142361search4turn142361search6

The pre-materialization profile enforces:

- maximum 200,000 regular-file blobs;
- maximum 64 MiB for one blob;
- maximum 768 MiB for total blob bytes;
- maximum 300,000 total tree entries;
- maximum 4,096 UTF-8 bytes per path;
- maximum 64 MiB aggregate path bytes.

The live Security Kernel candidate tree measured approximately 415 MiB with a largest blob
of approximately 49 MiB, leaving deterministic headroom under the registered total bound.

The candidate source identity is hashed with unambiguous length-framed records that also
include the extracted file mode:

```text
u32(mode_bits) || u64(path_bytes_length) || path_bytes ||
u64(content_bytes_length) || content_bytes
```

The source digest is captured as trusted step state before dependency acquisition and is
recomputed after candidate execution. Any mutation fails qualification.

### Candidate static trust-surface audit

The candidate Security Kernel implementation is checked independently of its own CI
workflow. The profile currently requires, among other invariants:

- opaque/private verification, revocation, and authority propositions;
- compiler-visible `#[must_use]` boundaries on verified capabilities and signature results;
- private `SignedCapability` trust-bearing fields with read-only accessors;
- bounded security JSON input and bounded action/identifier wire decoding;
- constructor-gated `Capability` and `AuthorizationRequest` deserialization;
- `deny_unknown_fields` on security wire types;
- no public trust-construction helpers or trust-sensitive SecurityEvent fields;
- exact Security Kernel package identity `mycelix-bridge-common`, edition 2024;
- Cargo.lock format v4;
- exact crates.io registry provenance and canonical 64-hex SHA-256 checksums;
- no candidate-controlled Cargo source overrides, `[patch]`, `[replace]`, or relevant
  Cargo configuration files;
- if a candidate supplies a workflow at the legacy
  `.github/workflows/security-kernel-qualification.yml` path, it is treated purely as
  untrusted data; S1 does not depend on its existence, execution, or result, and rejects
  privileged `pull_request_target` use or secret references in that file.

A static predicate exercise was performed against Security Kernel PR #3822 exact head
`673de287aa6923a16bcb22c0970dd165573ea8e8`. The inspected kernel/events/manifest/lockfile/
workflow surfaces satisfied the current registered static predicates. This is structural
evidence for that exact candidate tree, not a runtime qualification PASS.

### Dependency trust zone

Dependency acquisition is a separate constrained container:

- receives only the committed bridge `Cargo.toml` and `Cargo.lock`;
- non-root;
- dropped Linux capabilities;
- `no-new-privileges`;
- default seccomp;
- private namespaces;
- bounded CPU, memory, PIDs, descriptors and temporary storage;
- no GitHub/OIDC/runtime tokens;
- no Docker socket;
- outbound network is permitted only in this dependency-acquisition phase.

The lockfile is vendored with Cargo `--locked`. The generated vendor configuration is
required to be exactly the registered local-directory replacement and contains no
credentials. The vendor tree is hashed with the same length-framed record format and is
mounted read-only for candidate execution.

### Candidate execution zone

Every candidate gate runs in a fresh disposable container using the immutable Rust image:

`docker.io/library/rust@sha256:9af5f5f37d3035dd18d216e348e946ff1fc8c7fa7998c7443cedfe880231110d`

The current registered sandbox controls include:

- explicit `linux/amd64`;
- active seccomp filtering is observed in the container (`Seccomp=2` with filters present);
- KVM, `/dev/mem`, `/dev/kmem`, and `/dev/kmsg` device surfaces are absent;
- network disabled;
- read-only root filesystem;
- non-root user;
- all Linux capabilities dropped;
- `no-new-privileges`;
- default seccomp;
- bounded CPU, memory, PIDs, descriptors and wall time;
- private PID/IPC/cgroup namespaces;
- writable state limited to bounded tmpfs locations;
- candidate source and vendored dependencies mounted read-only;
- no Docker socket;
- no GitHub token, OIDC request token, or Actions runtime token;
- build target isolated in its own tmpfs.

The registered Rust identity is Rust 1.99.0 with compiler commit
`b940084d7eb6a299eb4bfeb8e34901bc051e7ac4`.

Qualification gates are rustfmt, default-feature tests, identity-feature tests, and Clippy
for both default and identity feature sets. All candidate Cargo gates use the committed
lockfile and frozen/offline dependency resolution.

### Receipt

S1 emits a non-authoritative receipt containing at least:

- candidate repository/PR/SHA/tree;
- candidate source file count and total bytes;
- length-framed source digest;
- Cargo.lock format and SHA-256;
- vendor digest;
- immutable sandbox image digest;
- sandbox control profile;
- trusted S0 workflow reference and SHA;
- Rust version/compiler commit;
- `source_event=pull_request_target`;
- `qualification_pass=true`.

The receipt is evidence only. S2 never treats candidate-produced receipt text as the source
of truth.

## S2 — trusted result verifier

`.github/workflows/security-kernel-trusted-result-verifier.yml` runs from protected
`main` on completion of the trusted S0 workflow.

S2 independently verifies:

- the triggering run is the exact S0 `workflow_run` event and completed successfully;
- the S0 workflow reference is exactly the default-branch trusted dispatcher;
- the S0 workflow blob executed by that run matches the registered S0 profile;
- the same-commit S1 workflow blob matches the registered S1 profile;
- the S0 workflow commit is an ancestor of current protected `main`;
- the current PR number/head SHA remain exact;
- the S0 run has exactly the expected two-job topology;
- jobs are fetched from the exact `run_attempt`, avoiding latest-attempt confusion;
- the resolver job completed successfully;
- the unique reusable S1 job completed successfully;
- every required S1 gate completed successfully.

S2 is deliberately read-only. It does not publish mutable repository status and does not
execute candidate code.

## Evidence ceiling

A verified PASS under this profile means that the registered gates passed for one exact
candidate commit under the registered S0/S1/S2 mechanism.

It does **not** mean:

- formal verification;
- absence of implementation vulnerabilities;
- protection against runner, Docker daemon, or Linux kernel escape;
- protection against GitHub platform compromise;
- independent human security review;
- product correctness;
- runtime authorization.

The current host/container boundary remains trusted infrastructure. Stronger microVM/KVM
isolation is tracked in **#4152**.

## Deployment gates

Before this mechanism is treated as authoritative:

1. install S0, S1, and S2 on protected `main`;
2. establish the applicable `pull_request_target` Actions event policy required by
   **#4195** before November 2, 2026;
3. enforce the branch/ruleset review and no-bypass governance recorded in **#4196**;
4. execute a real candidate run and capture the complete S0 -> S1 -> S2 attempt;
5. retain exact run/attempt, workflow-identity, source-digest, lockfile, vendor, sandbox,
   and gate evidence.

Until then, queued CI or a committed workflow definition remains **unverified**, not PASS.

## Related hardening

- **#4152** — stronger microVM/KVM execution boundary for hostile candidate code.
- **#4159** — immutable trusted invocation evolution; the adopted same-commit reusable S1
  model is now anchored by a metadata-only PR-target S0.
- **#4195** — explicit public-repository Actions policy for the metadata-only
  `pull_request_target` trust root.
- **#4200** — policy-independent scheduled fallback for qualification liveness if
  `pull_request_target` becomes unavailable.
