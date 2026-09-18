# JIT Qualification Execution Contract v0.1

Status: design freeze candidate (JIT-0 only)

This contract defines a bounded emergency execution plane for exact-subject qualification when GitHub-hosted runner assignment is unavailable. It does not create, register, or authorize a runner by itself.

## Scope

JIT-0 freezes:
- the exact execution-envelope fields and validation order;
- the runner lifecycle state machine;
- the execution-plane failure taxonomy;
- evidence/containment separation;
- activation, security, and claim-ceiling invariants.

JIT-0 does not:
- register GitHub runners;
- add `runs-on: self-hosted` to ordinary workflows;
- execute public pull-request code;
- satisfy any ASSURE product qualification;
- grant merge, deployment, constitutional, governance, or political authority.

## Normative rules

### Envelope

JIT-001 — Exact subject only.
A qualification envelope MUST bind an immutable repository, commit OID, tree OID, and expected predecessor when the profile requires one. Branch names, tags, PR heads, and other mutable references MUST NOT substitute for these identifiers.

JIT-002 — Trusted workflow identity.
The envelope MUST bind the default-branch qualification workflow by repository path, immutable workflow commit, and workflow-file SHA-256. Candidate product code MUST NOT choose the workflow definition or runner group.

JIT-003 — Short-lived authorization.
The envelope MUST contain an expiry and unique 256-bit nonce. Expired envelopes and previously consumed nonces MUST fail closed.

JIT-004 — Operator authorization.
The envelope MUST bind an operator authorization artifact by SHA-256. Controller policy MUST independently decide whether that authorization is trusted for the requested qualification profile.

JIT-005 — Exact technical identities.
The envelope MUST bind the qualification-profile name and SHA-256, qualified-parent receipt SHA-256, specification root, coverage root, corpus root, dependency lock root, toolchain identity, runner image root, controller-policy root, network-policy root, allowed-command-plan root, target runner-group identifier, qualification-receipt schema, and containment-receipt schema.

JIT-006 — Canonical envelope bytes.
The normative envelope serialization is UTF-8 JSON conforming to RFC 8785 JSON Canonicalization Scheme (JCS). The envelope digest is SHA-256 over those canonical bytes. Non-canonical input MUST NOT be treated as the normative envelope representation. Numeric fields MUST remain within the exact integer range representable by I-JSON/JCS; v0.1 therefore caps Unix-microsecond expiry at 2^53-1.

JIT-007 — Profile-required predecessor.
The only v0.1 profile is `assure-002b-l0-v1`; its expected predecessor is mandatory and MUST be an exact Git commit OID. Null or inferred predecessor semantics are forbidden in v0.1.

JIT-008 — Canonical workflow path.
The workflow path MUST be a normalized repository-relative POSIX path under `.github/workflows/`. Dot segments, parent segments, repeated separators, backslashes, NULs, and absolute paths are forbidden.

JIT-009 — Profile consistency.
`profiles/assure-002b-l0-v1.toml` is the only v0.1 qualification-profile manifest. An envelope is not admissible merely because it satisfies JSON Schema: every profile-bound field MUST exactly equal that manifest, and the envelope MUST bind the manifest's SHA-256. A profile mismatch fails before runner registration.

### Activation

JIT-010 — No automatic public-PR failover.
An arbitrary branch, fork, PR, workflow input, runner label, or contributor action MUST NOT activate the JIT execution plane.

JIT-011 — External controller.
Runner provisioning/JIT registration MUST be initiated by an operator-controlled controller outside the GitHub Actions job being rescued. The fallback MUST NOT depend on a GitHub-hosted job successfully starting.

JIT-012 — Default-branch dispatch.
If GitHub Actions is used as dispatcher, the qualification workflow MUST be an operator-controlled `workflow_dispatch` workflow present on the repository default branch. `pull_request_target` MUST NOT execute candidate product code.

JIT-013 — Inputs are data.
Dispatch/envelope inputs MUST be parsed and validated as data. They MUST NOT be interpolated as shell syntax, command fragments, environment-variable names, runner labels, arbitrary URLs, or arbitrary file paths.

### Runner isolation

JIT-020 — One job, one disposable environment.
A JIT runner MUST execute at most one qualification job and MUST run in a fresh disposable VM or equivalently destroyed isolation boundary.

JIT-021 — No persistent workspace.
The qualification environment MUST NOT inherit a reusable repository workspace, host bind mount, Docker socket, SSH agent, cloud credential, signing key, or deployment credential.

JIT-022 — Read-only repository authority.
The subject execution environment MUST have no repository write authority. `actions/checkout` or equivalent checkout MUST not persist write credentials.

JIT-023 — No OIDC privilege.
The initial profile MUST NOT grant `id-token: write`, deployment environments, package publication authority, or production secret access.

JIT-024 — Subject re-binding before execution.
Inside the clean runner, the launcher MUST re-verify the exact commit, tree, expected predecessor, qualification workflow identity, dependency lock root, spec root, toolchain identity, runner image root, network policy root, and allowed-command-plan root before subject-controlled commands execute.

JIT-025 — Immutable image identity.
Mutable image names such as `latest` are insufficient. Qualification evidence MUST bind the immutable VM/base-image or Nix/system closure digest and the GitHub Actions runner application version.

JIT-026 — Ephemeral runner credential only.
The disposable runner may receive only the single-use/short-lived credential material required for that JIT job. Long-lived controller credentials, PATs, organization administration credentials, and reusable runner-registration credentials MUST remain outside the subject environment.

JIT-027 — Runner-group binding.
The envelope MUST name the intended qualification-only runner group. The controller and runner-side binder MUST reject execution when GitHub routes the job through a different group or an unqualified generic self-hosted pool.

### Network

JIT-030 — Deny by default.
The initial fallback profile MUST use deny-by-default egress. Any permitted destination or preparation phase MUST be represented in the network-policy artifact whose digest is bound by the envelope.

JIT-031 — Separate preparation from qualification.
If dependency/toolchain acquisition requires network access, acquisition SHOULD occur in a separately evidenced preparation phase. The qualification phase SHOULD run locked/offline whenever the profile permits it.

### Lifecycle and containment

JIT-040 — Monotone lifecycle.
Lifecycle transitions MUST follow `state-machine.toml`. No transition may return to an earlier reusable state.

JIT-041 — Quarantine on abnormal transition.
Any lifecycle invariant failure MUST transition to `Quarantined` and then attempt `Destroyed`. A quarantined runner MUST NOT execute another job.

JIT-042 — External log preservation.
Qualification logs and receipt material MUST be exported outside the disposable VM before destruction. Missing required logs is a containment failure.

JIT-043 — Destruction is evidence-bearing.
A technical PASS is not final qualification evidence until destruction/cleanup evidence is recorded. Destroying a runner registration without destroying/resetting the machine is insufficient.

JIT-044 — No second-job reuse.
Any evidence that the environment accepted or could accept a second qualification job under the same prepared instance invalidates containment for the protected profile.

### Evidence semantics

JIT-050 — Separate results.
`QualificationResult` and `ExecutionContainmentResult` are distinct. Product PASS plus containment FAIL MUST NOT be promoted to qualified evidence.

JIT-051 — Execution provenance is not authority.
Runner identity, image provenance, logs, and containment evidence describe how qualification ran. They do not create constitutional, governance, merge, deployment, or political authority.

JIT-052 — Same claim ceiling.
A JIT execution may establish no broader proposition than the corresponding qualification profile would establish on a GitHub-hosted runner.

JIT-053 — Equivalence before ASSURE use.
Before JIT execution may qualify ASSURE subjects, an equivalence campaign MUST run the same frozen non-production subjects through GitHub-hosted and JIT execution and compare all deterministic qualification outputs and failure classes. Unexplained disagreement blocks promotion.

JIT-054 — Dual-receipt promotion.
JIT execution MUST preserve the ordinary narrow ASSURE qualification receipt as the product-gate result and MUST emit a separate containment receipt conforming to `containment-receipt.schema.json`. The containment receipt MUST bind the SHA-256 of the ordinary qualification receipt and the execution-envelope SHA-256. Promotion requires an ordinary qualification PASS and containment PASS whose subject, envelope, and qualification-receipt bindings agree exactly. Neither receipt may silently absorb or replace the other's semantics.

JIT-055 — Containment receipt consistency.
A containment PASS MUST contain an empty JIT failure-code set. A containment FAIL MUST contain at least one registered JIT failure code. `destroyed_at_unix_micros` MUST be greater than or equal to `started_at_unix_micros`; this relational timestamp invariant is normative even though JSON Schema cannot express it directly.

### Replay/freshness

JIT-060 — Nonce consumption.
A controller MUST atomically record nonce consumption before JIT registration. Failure to persist nonce consumption MUST fail closed.

JIT-061 — Expiry.
The controller MUST verify expiry immediately before registration and again before subject execution. A job that starts after expiry MUST not execute subject-controlled commands.

JIT-062 — No implicit renewal.
An expired or consumed envelope cannot be renewed by retrying the workflow. A new envelope requires a new nonce and authorization artifact.

## Claim ceiling

A conforming JIT-0 implementation can at most establish that its execution-plane controls satisfy this contract for the tested campaign.

JIT-0 itself does not establish:
- ASSURE-002B PASS;
- graph validity;
- evidence truth or admissibility;
- effective authority;
- political or constitutional legitimacy;
- moral truth;
- repository merge authorization;
- deployment authorization;
- superiority of self-hosted/JIT execution.

## JIT-0 exit gate

JIT-0 is complete only when:
1. this spec root is reproducibly verified;
2. the envelope schema, containment-receipt schema, profile, and state machine are internally consistent;
3. every failure code is unique and append-only within v0.1;
4. no executable runner registration or public-PR self-hosted path is introduced by the tranche;
5. #1671 remains the controlling implementation issue for JIT-1 and later stages.
