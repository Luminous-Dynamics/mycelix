# Lifecycle Current Executor Authority — Normative Integration Contract

Status: **binding v0.1 provider contract; verifier and provider remain unprovisioned in the checked-in DNA**

This document sharpens the meaning of the existing lifecycle `ExecutionDomain.executor_authority_ref` without changing the lifecycle wire format.

The lifecycle verifier remains a composition boundary. It does not choose an executor, issue an institutional grant, validate a delegation by itself, select a revocation generation, or mint execution authority.

## Deterministic lifecycle-facing executor reference

For live admission, the only valid v0.1 meaning of `executor_authority_ref` is:

`current-executor:<CURRENT_EXECUTOR_AUTHORITY_PROFILE>:<64-hex digest>`

where `<64-hex digest>` is the exact `QualifiedCurrentExecutorAuthority.current_authority_digest` produced by `mycelix-governance-current-executor-authority` and projected by `mycelix-governance-executor-provider-contract`.

A caller-provided string, designation record reference, grant ID, delegation proof reference, verification invocation reference, role, score, or principal ID is not an executor-authority identity.

## Required provider

`governance_executor_authority_verifier` must independently resolve the exact authority inputs and current-state evidence required by the pure stack and then invoke the executor-provider contract.

The positive provider result must correspond to `VerifiedCurrentExecutorAuthorityReceipt` semantics and must bind at least:

- exact proposal ID;
- exact executable action digest/profile;
- exact threshold-authorization reference;
- exact stable threshold-authorization identity digest/profile;
- exact executor principal;
- deterministic current executor authority ref;
- exact current executor authority digest/profile;
- exact lineage-bound semantic executor authority digest/profile;
- exact canonical authority-grant ID/digest/profile;
- exact institution/jurisdiction/rulebook;
- exact capability scope;
- exact generation-bound freshness bundle digest/profile;
- semantic authority validity ceiling;
- bounded current reuse horizon; and
- provider verification provenance.

The provider receipt is deserializable ABI data, not self-authenticating authority. The lifecycle verifier must obtain it directly from the named local authority provider and validate it.

## Threshold cross-provider equality

The threshold verifier and executor provider must agree on more than the textual threshold authorization reference.

They must return the **same threshold-authorization identity digest/profile** for the exact authorization being consumed.

Same textual threshold reference with different signing-policy, committee/key epoch, action digest/profile, signature identity, authority lifetime, or other identity-bearing threshold semantics is denial.

The lifecycle verifier must not accept:

`threshold_ref == threshold_ref`

as sufficient when:

`threshold_identity_A != threshold_identity_B`.

## Exact proposal/action equality

The executor provider must bind the same proposal ID and executable action digest/profile already verified by:

1. the immutable execution-plan verifier;
2. constitutional execution preflight; and
3. threshold authorization.

Any disagreement is denial.

## Generation-sensitive domain identity

The deterministic executor ref makes the existing lifecycle domain sensitive to current authority without adding new lifecycle fields.

Any change that changes the exact current-executor authority digest changes `executor_authority_ref`, including at least:

- executor grant revocation, supersession, or re-authorization;
- threshold/signing-policy/committee/key dependency revocation or epoch replacement;
- executor designation replacement;
- delegation policy revocation/re-authorization;
- delegation or intermediate grant generation changes;
- delegated lineage changes; or
- another dependency-closure generation change represented by the current-authority provider.

A domain reconstructed before such a change is stale and must fail exact-current-domain comparison.

## Same-generation refresh is not new authority

Provider verification references, provider invocation timestamps, and bounded lease horizons are dynamic evidence.

If all semantic identities and current generations remain unchanged, re-verification may refresh those fields without changing the deterministic `executor_authority_ref` or lifecycle domain digest.

A refreshed proof is not a new grant.

## Effect-safety binding

The effect-safety provider already receives the exact lifecycle `executor_authority_ref`.

Under this contract that reference identifies the exact generation-bound current executor authority. Therefore effect-safety approval is also invalidated when current executor authority changes.

The safety provider must not translate a stale executor ref into approval for a refreshed executor generation.

## Ready and Claim are live checks

`ReadyAuthorized` and `Claimed` are live-admission states.

Their qualification must reconstruct the current execution domain and therefore require the current deterministic executor authority ref.

A previously valid Ready event does not provide a durable lease across:

- executor authority revocation;
- threshold authority revocation;
- constitutional change;
- delegation-lineage change; or
- safety-policy change.

The final pre-effect boundary must re-run current qualification.

## Historical verification is separate

Historical/as-of verification must reconstruct the authority generations effective at the event time.

It must not require today's `executor_authority_ref` to equal the historical domain's ref, and a historical positive result can never authorize a new current claim or effect.

Later ordinary expiry or later revocation prevents future use without rewriting a legitimately authorized historical event. A revocation effective before the event, or proof that the original authority/signature was invalid, may invalidate the historical event.

## Failure semantics

Any of the following is denial for live execution:

- executor provider absent/unavailable;
- decode failure;
- provider protocol mismatch;
- malformed deterministic executor ref;
- future verification time;
- expired provider lease;
- stale/revoked/superseded current generation;
- threshold ref mismatch;
- same ref but different threshold identity digest/profile;
- proposal/action digest/profile mismatch;
- executor principal mismatch;
- institution/rulebook/capability mismatch;
- malformed canonical grant identity;
- malformed lineage/current-authority/freshness profile; or
- inability to prove exact effect-safety binding.

There is no score-based, creator-based, Guardian-based, caller-based, or best-effort fallback.

## Packaging state

This contract does not provision `governance_executor_authority_verifier` or `governance_execution_lifecycle_verifier` into the binding governance DNA.

External effects remain disabled until the provider implementation, lifecycle integration, historical verifier, and adversarial qualification suites are all green.
