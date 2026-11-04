# Pulse authorization lifecycle — bounded model-check receipt

**Result:** PASS for the stated finite model and exact model-check run  
**Workflow:** [Pulse Authority Lifecycle Model, run #18](https://github.com/Luminous-Dynamics/mycelix/actions/runs/41834410638)  
**PR head checked:** `39bbdd56a5f405470b66411a1481954640645a38`  
**PR base at run time:** `9e12dd46bdd8371e8ed9626a17bc06f21c141a52`  
**Merge checkout SHA:** `6ecbbfd908fd87bbefdb3dc75b79a898517d3338`  
**TLC:** 1.8.0 / build `2026.08.11.125311`  
**TLC JAR SHA-256:** `ab323b79802aedc3203b3f9af37c6aca3ed43f4e0225b36f2aa77b26de46c05f`  
**Artifact:** [run artifact](https://github.com/Luminous-Dynamics/mycelix/actions/runs/41834410638/artifacts/13323942606), ID `13323942606`, ZIP SHA-256 `8e6a96858320004d4fbc6ca58f7eaa928a999a691471700926a27e65e5beff8d`  
**Java:** Temurin OpenJDK 21.0.13+9

## Exact results

| Configuration | Result | States generated | Distinct states | Invariant / expected failure |
|---|---|---:|---:|---|
| Safe `PulseAuthorityLifecycle.cfg` | PASS | 83,941 | 10,249 | No invariant error; state queue exhausted |
| Projection mutant `PulseAuthorityLifecycle-mutant.cfg` | Expected counterexample found | 419 | 199 | `NoProjectionAuthority` violated |
| Grant-idempotency mutant `PulseAuthorityLifecycle-grant-idempotency-mutant.cfg` | Expected counterexample found | 59 | 47 | `GrantIdentityInjective` violated |

The negative-control workflow steps pass only when TLC exits non-zero and reports the invariant that the mutant is intended to violate. The workflow does not accept a generic parser failure as a valid mutant detection.

## Repeat run on updated PR head

The model workflow was rerun as **run #36** after the PR documentation and issue cross-links were updated:

- Run: https://github.com/Luminous-Dynamics/mycelix/actions/runs/41853504791
- PR head: `d31079d4606e05268e33d4ea2bb310bc6bb99a93`
- Merge checkout: `dec4aeef` (merge of that head into the same base)
- Artifact ID: `13336008629`
- Artifact ZIP SHA-256: `421090d8fee0115e78b3ce436ebfa5399bd0c6f6a6d24071dd899381af09de41`

All six model phases reproduced the earlier outcomes: safe authority model 83,941 generated / 10,249 distinct states with no invariant error; projection mutant caught by `NoProjectionAuthority`; grant-idempotency mutant caught by `GrantIdentityInjective`; safe mailbox model 37 generated / 20 distinct states with no invariant error; proposed-state mutant caught by `OnlyOriginalOwnerMayUpdate`; field-smuggling mutant caught by `ProtectedFieldsRemainImmutable`.

This is a repeated run of the same model/configuration logic, not an expansion of the bounds. It increases confidence in workflow reproducibility but does not increase the scope of the model's claims.

## Repeat run after documentation update

The same six model phases were rerun in [workflow run #60](https://github.com/Luminous-Dynamics/mycelix/actions/runs/41857220290) against PR head `34b1d695b8ce7d5bc878ee9d3025fa738514f1d2` (merge checkout `8e4943ff`).

All six outcomes reproduced:
- authority safe: 83,941 generated / 10,249 distinct, no invariant error;
- projection mutant: `NoProjectionAuthority`;
- grant retry mutant: `GrantIdentityInjective`;
- mailbox safe: 37 / 20, no invariant error;
- proposed-state mutant: `OnlyOriginalOwnerMayUpdate`;
- field-smuggling mutant: `ProtectedFieldsRemainImmutable`;
- checkpoint safe: 27 / 17, no invariant error;
- checkpoint bypass mutant: `NoFalseFreshAuthority`.

The workflow artifact is ID `13337303730`, ZIP SHA-256 `17da4b4450375fd6c262aa59f80d680954f297a2a8cd4a55b8a2b542995ca36a`. This is reproducibility evidence for the same finite models and bounds, not new model coverage.

## Counterexample interpretations

### Projection-as-authority mutant

TLC found the schedule:

1. Issue the assigned grant.
2. Deliver the recipient claim.
3. Revoke the host grant.
4. Leave the application/DHT projection stale-active.
5. Invoke the protected function as the assigned grantee.

The mutant incorrectly allows the call because it treats the stale active projection as authority. The safe model bases the host decision on the current host grant and denies the post-revocation call. This validates the *encoded finite invariant*; it is not evidence of a real conductor run.

### Duplicate-grant-on-retry mutant

TLC found the schedule:

1. Create one grant for a logical operation.
2. Drop the acknowledgement, leaving the caller outcome unknown.
3. Retry the same logical request without idempotency.
4. Create a second host grant for the same operation while the original remains live.

The operation-to-grant mapping is overwritten, so `GrantIdentityInjective` detects the orphaned original grant. This validates the encoded idempotency counterexample; it does not demonstrate that a production deployment has observed a duplicate grant.

## Boundaries

The model is finite and deliberately small: two grants, two logical operations, three actors (grantor, grantee, outsider), and at most two issuance attempts per operation. It abstracts host capability enforcement, claim presence, domain authorization, public application state, projection freshness, acknowledgement loss, retry and revocation. It does not model Holochain implementation internals, cryptographic verification, source-chain validation, multi-conductor DHT convergence, arbitrary partitions, Byzantine consensus, unbounded runs or production performance.

The constant-configuration checks also emitted TLC's informational warning that constant-level formulas are being asserted as invariants. They evaluated true; this is a style warning, not a failed check.

**Claim ceiling:** the result is a bounded model-check of this exact TLA+ source/configuration set with the pinned TLC build. It is not a Sweettest, conductor test, proof of general linearizability, proof of unbounded crash consistency, or production qualification.

## Reproduction

The exact workflow is versioned at `.github/workflows/pulse-authority-model.yml`. It downloads the immutable build-specific TLC artifact, verifies the SHA-256 before execution, runs the safe config and both expected-failure mutants, and uploads the logs and receipt artifact for 30 days.

The next model expansion should add concurrent overlapping calls/revocations, independent grant coexistence/revocation, multi-conductor validator acceptance/rejection, competing root forks and crash boundaries. Keep the same rule: the safe configuration must preserve invariants, while a deliberately weakened mutant must be caught by an invariant-specific counterexample.
