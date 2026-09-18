# JIT-0 Conformance Checklist

This checklist is design-level only. Passing it does not create or qualify a runner.

## Envelope
- [ ] JSON Schema parses as Draft 2020-12.
- [ ] Repository is fixed to `Luminous-Dynamics/mycelix` for v0.1.
- [ ] Subject commit/tree are immutable 40-hex Git object IDs.
- [ ] Expected parent is mandatory for v0.1 `assure-002b-l0-v1`; null/inferred predecessor semantics are rejected.
- [ ] Workflow path, workflow commit, and workflow-file SHA-256 are all bound.
- [ ] The initial qualification profile is only `assure-002b-l0-v1` and its exact manifest SHA-256 is bound.
- [ ] Subject commit/tree, predecessor, parent receipt, spec root, coverage root, corpus root, dependency lock, toolchain, both receipt schemas, and manual obligations match the frozen profile manifest exactly.
- [ ] Spec, dependency lock, runner image, network policy, and command plan are all SHA-256 bound.
- [ ] Expiry is explicit Unix microseconds and no greater than `2^53-1`, preserving exact JCS number semantics.
- [ ] Nonce is exactly 256 bits.
- [ ] Operator authorization is referenced by SHA-256.
- [ ] Controller policy is SHA-256 bound.
- [ ] Qualification-only runner-group identifier is explicit and validated.
- [ ] Workflow path is canonical and rejects both leading and nested dot/parent segments, repeated separators, backslashes, and absolute-path syntax.
- [ ] Normative bytes are RFC 8785 JCS and envelope digest is SHA-256 over those bytes.

## Admission
- [ ] No public PR, fork, branch name, tag, or mutable ref can activate JIT.
- [ ] Controller exists outside the rescued GitHub Actions job.
- [ ] GitHub dispatch, if used, is `workflow_dispatch` on default branch.
- [ ] `pull_request_target` is not used to execute candidate code.
- [ ] Inputs are parsed as data and cannot become shell syntax/runner labels/arbitrary URLs.

## Isolation
- [ ] Fresh disposable VM or equivalent destroyed isolation boundary per job.
- [ ] At most one job per JIT registration and prepared environment.
- [ ] No host workspace/bind mount/Docker socket/SSH agent.
- [ ] No repository write token, deployment secret, signing key, cloud credential, or OIDC minting.
- [ ] Only single-use/short-lived JIT job credential material enters the disposable runner; long-lived controller/PAT/admin credentials remain outside.
- [ ] Exact subject and all roots are reverified inside the clean runner before commands execute.
- [ ] Runner image identity is immutable and evidence-bound.

## Network
- [ ] Egress policy is deny-by-default.
- [ ] Online preparation is separate/evidenced where required.
- [ ] Qualification uses locked/offline execution when the profile permits it.

## Lifecycle
- [ ] Only transitions in `state-machine.toml` are accepted.
- [ ] Any abnormal transition quarantines the environment.
- [ ] Quarantined environments cannot execute a second job.
- [ ] Logs and both receipts are exported before destruction.
- [ ] Destruction evidence is mandatory.
- [ ] Product PASS + containment FAIL cannot become qualified evidence.

## Dual receipt
- [ ] The ordinary ASSURE qualification receipt remains in its existing schema and claim ceiling.
- [ ] A separate containment receipt validates against `containment-receipt.schema.json`.
- [ ] The containment receipt binds the ordinary qualification receipt SHA-256 and execution-envelope SHA-256.
- [ ] Subject commit/tree agree across envelope, qualification receipt, and containment receipt.
- [ ] Promotion requires `qualification_result = PASS` and `containment_result = PASS`.
- [ ] `containment_result = PASS` requires an empty JIT failure-code list.
- [ ] `containment_result = FAIL` requires at least one registered JIT failure code.
- [ ] Runtime validation requires `destroyed_at_unix_micros >= started_at_unix_micros`.
- [ ] Containment failure codes are JIT execution-plane codes only and do not redefine ASSURE product failure semantics.

## Promotion
- [ ] JIT-1 local disposable harness passes.
- [ ] JIT-2 one-job GitHub JIT registration passes.
- [ ] JIT-3 adversarial campaign passes.
- [ ] JIT-4 hosted/JIT equivalence campaign passes.
- [ ] Only then may JIT-5 execute an ASSURE qualification during hosted-runner unavailability.
