# Mycelix Governance Executor Provider Contract v0.1 — Normative Invariants

Status: **pure runtime-provider projection contract; not a Holochain authority source**

This crate defines how the exact current executor authority from the pure authority stack is projected into the existing lifecycle v0.1 provider ABI.

It does not discover grants, resolve DHT state, verify signatures, select current generations, or execute effects.

## 1. Existing lifecycle ABI is preserved

Lifecycle v0.1 already carries `executor_authority_ref` as an opaque string.

No protocol-version change is required merely to strengthen the provider-side meaning of that opaque field.

The normative v0.1 meaning is now:

`current-executor:<CURRENT_EXECUTOR_AUTHORITY_PROFILE>:<64-hex current authority digest>`

where the digest is produced only by `mycelix-governance-current-executor-authority`.

## 2. Caller-selected executor authority refs are forbidden

`executor_authority_ref` is derived by `executor_authority_ref(digest, profile)`.

A runtime provider MUST NOT:

- echo a caller-provided authority ref;
- reuse a designation-record ref as the final lifecycle authority identity;
- substitute a grant ID, delegation proof ref, threshold ref, verification ref, score, role, or principal ID; or
- preserve the same authority ref after the exact current-authority digest changes.

## 3. Provider qualification re-runs exact current authority

`qualify_executor_provider_receipt` calls `qualify_current_executor_authority` from PR #82 using the exact:

- threshold authorization wrapper;
- executor grant/proof receipt;
- executor designation/proof receipt;
- direct/delegated lineage evidence;
- grant current-freshness receipt;
- threshold current-freshness receipt; and
- executor current-freshness receipt.

The provider projection is therefore downstream of the entire semantic + revocation chain, not an alternative authority path.

## 4. Projection echoes exact authority-bearing semantics

`CurrentExecutorAuthorityProjection` exposes the exact:

- proposal;
- executable action digest/profile;
- threshold authorization ref and stable semantic identity digest/profile;
- executor principal;
- deterministic current executor authority ref;
- current executor authority digest/profile;
- lineage-bound semantic executor authority digest/profile;
- canonical executor-grant ID/digest/profile;
- institution/jurisdiction/rulebook;
- capability scope;
- generation-bound freshness bundle digest/profile; and
- semantic validity ceiling.

These fields exist so the lifecycle verifier can compare exact semantic bindings rather than trusting a provider label.

## 5. Deterministic current authority ref

The lifecycle-facing ref is a deterministic function only of:

- `CURRENT_EXECUTOR_AUTHORITY_PROFILE`; and
- exact `current_executor_authority_digest`.

It does not include verifier invocation timestamps, verification references, or lease horizons.

Therefore same-generation re-verification preserves the execution-domain identity.

Any semantic authority change, revocation/supersession generation change, or dependency-closure generation change alters the digest and therefore the ref.

## 6. Provider verification metadata is dynamic evidence

`verification_ref`, `verified_at_ms`, and `valid_until_ms` describe one provider invocation and its reuse ceiling.

They are not authority identity.

A provider receipt may refresh those values while preserving the same deterministic executor authority ref only when the exact current-authority digest/profile is unchanged.

## 7. Reuse validity can only shrink authority

`valid_until_ms` is the lease from PR #82 and MUST be no later than the semantic executor validity ceiling.

At or after the lease horizon, the lifecycle verifier must call the provider again.

Lease refresh never overrides revocation or generation changes.

## 8. Deserializable provider receipt is not self-authenticating authority

The wire projection/receipt derives `Deserialize` because it is a runtime ABI.

That does not make arbitrary application data authoritative.

A lifecycle verifier MUST obtain the receipt by directly calling the independently authoritative executor provider and MUST validate the returned protocol, projection, deterministic authority ref, verification window, and exact requested proposal/action/threshold bindings.

## 9. Lifecycle domain becomes generation-sensitive without ABI churn

Because `ExecutionDomain.executor_authority_ref` stores the deterministic current authority ref:

- grant revocation/re-authorization;
- signing/threshold dependency revocation;
- executor designation replacement;
- delegation policy/grant/delegation revocation;
- lineage-generation change; or
- other PR #82 current-authority changes

change the execution-domain identity on the next live reconstruction.

A stale Ready/Claim domain therefore fails exact-current-domain comparison even though immutable historical records remain auditable.

## 10. Effect-safety binding remains exact

The effect-safety provider already receives and echoes the lifecycle `executor_authority_ref`.

Under this contract, it therefore binds safety approval to the exact generation-bound current executor authority rather than merely a designation record.

A refreshed executor generation requires a fresh domain/safety qualification.

## 11. Historical verification is separate

The deterministic current authority ref is for **live** admission.

Historical/as-of verification must reconstruct the authority generations effective at the historical event time and must not require today's current executor ref to equal the historical one.

Historical validity never grants new current execution authority.

## 12. Fail-closed runtime integration

A future `governance_executor_authority_verifier` must resolve all required records/proofs/current-state evidence itself and invoke this contract.

Missing provider, lookup failure, ambiguous current state, revoked/superseded dependency, stale lease, malformed profile, mismatched action/threshold/domain binding, or invalid deterministic authority ref must deny.

External effects remain disabled.
