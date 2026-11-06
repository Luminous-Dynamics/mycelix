# Pulse trust-root lifecycle — bounded model-check receipt

**Result:** PASS for the finite abstract model and exact model-check run  
**Workflow:** [Pulse Authority Lifecycle Model, run #327](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42260136191)  
**PR head checked:** 254556c4377833c976f2e9576c90bdd49cefb6d8  
**PR base at run time:** 380b4198cab8c5372fdef21fa9a60a00a848877f  
**Merge checkout:** 42bb97db7db850dbf68e64cfea5c016247f7a0bc  
**TLC:** 1.8.0 / build 2026.08.11.125311  
**TLC JAR SHA-256:** ab323b79802aedc3203b3f9af37c6aca3ed43f4e0225b36f2aa77b26de46c05f  
**Artifact ID:** 13524926321  
**Artifact ZIP SHA-256:** 5c5d68e7c80f9aa7adbcc1ae9e29266fe60e94b937f069d25711cd23b6685965

## Results

| Configuration | Result | States generated | Distinct states | Outcome |
|---|---|---:|---:|---|
| Safe pinned-root policy lifecycle | PASS | 8 | 4 | State queue exhausted; no invariant error |
| Self-asserted-root mutant | Expected counterexample | 3 | 3 | OnlyPinnedRootCanAuthorizePolicy violated |
| Policy-rollback mutant | Expected counterexample | 3 | 3 | PolicyEpochNeverRollsBack violated |
| Revoked-key acceptance mutant | Expected counterexample | 3 | 3 | NoRevokedKeyAcceptedUnderCurrentPolicy violated |

The negative-control workflow steps pass only when TLC exits non-zero and the expected invariant name appears in the log. A syntax/parser failure does not count as a valid mutant detection.

## Model contract

The relying party's pinned root is an out-of-band initial state. A valid policy transition must:
- be authorized by the currently pinned root;
- extend the current policy digest/parent;
- advance the policy epoch;
- preserve the separation between policy authenticity and issuer-key role authorization;
- reject a currently revoked issuer key for a current-policy decision.

Historical signature validity is separate from current permission to rely on a statement. A signature made before a later key revocation may be historically authentic if there is trusted time and policy history, but it must not authorize new current decisions after revocation.

## Counterexamples

- **Self-asserted root:** the mutant accepts a new policy whose root is selected by the policy itself; TLC violates `OnlyPinnedRootCanAuthorizePolicy`.
- **Rollback:** the mutant permits an earlier/lower policy epoch; TLC violates `PolicyEpochNeverRollsBack`.
- **Revoked issuer:** the mutant accepts a currently revoked issuer for a current decision; TLC violates `NoRevokedKeyAcceptedUnderCurrentPolicy`.

## Boundaries

This is a finite abstract model only. It does not verify the Rust implementation, protect the actual filesystem storing the root pin, implement root rotation, threshold signatures, durable rollback-resistant state, trusted timestamps, historical compromise-time adjudication, SCITT COSE signatures or VDS proofs.

The subsequent Rust verifier increment is a separate implementation claim. Its tests and Clippy/format gates must pass against their own exact commit before the code is described as verified. Root pin deployment must be controlled out of band; a caller that supplies an attacker-controlled root pin can still create an attacker-controlled trust universe.

**Claim ceiling:** bounded model-check evidence for the exact TLA+ files/configuration and pinned TLC build. It is not a cryptographic implementation audit or a production root trust service.
