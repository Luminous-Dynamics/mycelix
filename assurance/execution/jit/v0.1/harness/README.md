# JIT-1A Offline Admission Harness

Status: exact-execution candidate only.

JIT-1A exercises the **admission/control-plane subset** of the frozen JIT-0 contract. It is deliberately not a VM harness and deliberately cannot register a GitHub Actions runner or execute qualification commands.

## Bound parent

- JIT-0 commit: `108a4505904d49f2f93924776e31e9def1c880ab`
- JIT-0 root: `78a8d7d525252b31ff24245514ab9288caec6bea56fba02349e4415a0a55546c`
- ASSURE-002B profile SHA-256: `9c9bea898f07a068e8d4c4be7d39adc81bf11a5b4ea9b9fc19d44b90c42a6a9b`
- canonical fixture-envelope SHA-256: `af6555d5eb0e5566095af2f6ecd2b73c2631a4a1a9ac01dd93b1a5e7ef2bfd43`

## What it implements

`controller_dry_run.py`:

1. runs the frozen JIT-0 contract checker;
2. requires the frozen JIT-0 root and profile hash;
3. accepts only the closed envelope field surface used by JIT-0 v0.1;
4. rejects duplicate JSON keys, floats, non-ASCII strings, arrays, nulls, booleans, unsafe integers, and non-canonical bytes;
5. verifies exact ASSURE-002B subject, predecessor, parent receipt, spec/coverage/corpus roots, dependency lock and toolchain through the frozen profile;
6. binds the test controller policy, workflow identity, runner group, deny-all network policy, empty command plan, descriptor-only runner image and operator-authorization fixture;
7. enforces expiry and maximum TTL;
8. can atomically consume a nonce in a caller-selected local ledger using `O_CREAT|O_EXCL`;
9. emits only `admission_result`, `authority = none`, and `execution = not_performed`.

`admit.py` is the public dry-run entry point. It adds a closed-world test-policy key surface and rejects unsafe fixture references before invoking the controller.

It has no runner-registration API, no GitHub API client, no shell-command plan executor, and no VM provisioning function.

## Mutation suites

`check_harness.py` covers:

- valid deterministic admission;
- one-time nonce consumption and replay rejection;
- non-canonical JSON;
- closed-world extra fields;
- expiry;
- subject mismatch;
- profile mismatch;
- workflow traversal;
- toolchain mismatch;
- runner-image mismatch;
- network-policy mismatch;
- command-plan mismatch;
- controller-policy mismatch;
- runner-group mismatch;
- operator-authorization mismatch.

`check_policy_guard.py` covers:

- canonical test policy admission;
- hidden/extra test-policy field rejection;
- fixture path traversal rejection.

## Pre-execution manifest

`JIT1A.lock` is frozen **before** exact execution. It is intentionally non-circular: it excludes only itself and binds every other JIT-1A payload file by exact Git blob OID plus a domain-separated SHA-256 manifest over sorted path/blob pairs.

`check_lock.py` recomputes those Git blob OIDs directly from file bytes without relying on the `git` executable, then reconstructs the payload manifest SHA-256.

Exact JIT-1A execution is valid only if `check_lock.py` passes before the mutation suites run.

## Claim ceiling

A JIT-1A PASS can establish only that the exact frozen offline admission/controller implementation accepts and rejects the registered envelope cases consistently with its frozen JIT-0 and payload-manifest bindings.

It does **not** establish:

- disposable VM isolation;
- GitHub JIT registration;
- network containment under a real guest;
- log export or destruction evidence;
- `ExecutionContainmentResult = PASS`;
- ASSURE-002B qualification;
- JIT-1 completion;
- JIT-5 eligibility;
- repository merge/deployment authority;
- constitutional, governance, political, or moral authority.

## JIT-1A exit gate

JIT-1A is complete only when all of the following are true for one exact frozen Git subject:

1. `JIT1A.lock` and `check_lock.py` verify the exact payload before execution;
2. the JIT-0 checker passes and returns the frozen JIT-0 root/profile identity;
3. the offline admission mutation suite passes;
4. the public policy-guard mutation suite passes;
5. nonce replay protection is exercised against a fresh local ledger;
6. the exact subject remains unchanged during execution;
7. the result is recorded against that exact commit/tree and payload-manifest identity.

Developer preflight against authoring copies is useful diagnostics but is not exact-subject qualification evidence.

Only after JIT-1A passes should JIT-1B add a **disposable NixOS VM** and test isolation/destruction. JIT-1 as a whole remains incomplete until JIT-1B passes.
