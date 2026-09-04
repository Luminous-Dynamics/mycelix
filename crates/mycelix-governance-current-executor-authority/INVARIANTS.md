# Mycelix Governance Current Executor Authority v0.1 — Normative Invariants

Status: **pure current-authority qualification kernel; no runtime authority source or effect executor**

This crate answers one narrow question:

> Is this exact executor authority not only semantically valid, but also current under the exact revocation generations that govern it now?

It deliberately re-runs the lineage-bound semantic qualifier from PR #81 before qualifying freshness. A previously serialized/remembered positive result is never enough.

## 1. Semantic authority and current authority are different facts

PR #81 proves exact executor semantics:

- exact proposal/action domain;
- exact threshold authorization;
- exact institutional grant;
- exact executor designation; and
- when delegated, exact current delegation lineage.

This crate adds a separate closed-set current-state proof over those same semantics.

Neither layer substitutes for the other.

## 2. Exact semantic qualification is re-run now

`qualify_current_executor_authority` receives the exact threshold wrapper, grant receipt, designation receipt, and delegation-lineage evidence and calls `qualify_lineage_bound_executor_authority` itself.

It does not accept a deserialized lineage-bound executor object from the caller.

This prevents freshness receipt A from being combined with a previously qualified semantic authority B.

## 3. Canonical executor grant freshness

The exact executor `AuthorityGrant` is canonicalized under:

`mycelix-authority-grant-v1-blake3-framed-semantic`

The current grant freshness subject is:

- kind: `AuthorityGrant`;
- namespace: exact institution;
- subject ID: exact grant ID; and
- identity: exact canonical grant digest/profile.

Same textual grant ID with changed holder, capability, rulebook, delegation lineage/proof source, or lifetime is a different freshness subject.

## 4. Stable threshold-authorization identity

This crate registers:

`mycelix-governance-threshold-authorization-v1-blake3-framed-semantic`

The identity binds the exact semantic threshold authority consumed by executor qualification, including:

- threshold authorization ref;
- proposal/institution/jurisdiction/rulebook/governing body/action class;
- exact action digest **and profile**;
- signing-policy ID/digest;
- committee ID/key digest/epoch;
- required/member/actual signer counts;
- signature algorithm;
- signature ID/ref;
- immutable signing-policy record ref;
- signature time; and
- semantic valid-until horizon.

Dynamic policy/crypto verifier invocation references and `verified_at_ms` are excluded from stable threshold identity.

## 5. Threshold freshness is dependency-closure state

`AuthoritySubjectKind::ThresholdAuthorization` current freshness MUST represent the entire authority dependency closure for that threshold authorization.

A current-state provider MUST advance generation or return `Revoked`/`Superseded` when any authority-bearing dependency ceases to authorize new execution, including at least:

- institutional signing policy revocation/supersession;
- governing-body authority loss;
- committee/key revocation;
- committee epoch replacement;
- threshold authorization revocation; or
- another rulebook-defined invalidation of that exact authorization.

The existence of a historically valid signature is not sufficient current authority.

## 6. Exact lineage-bound executor freshness

The executor freshness subject uses:

- kind: `ExecutorDesignation`;
- namespace: exact institution;
- subject ID: exact executor-designation record ref; and
- identity: PR #81's exact `QualifiedLineageBoundExecutorAuthority` digest/profile.

That semantic digest already commits the canonical grant, exact designation, threshold authorization ref and, for delegated authority, exact current delegation lineage.

A designation or executor authority replacement therefore requires a new current state.

## 7. Delegated authority remains current only through current lineage

For delegated grants, this crate re-runs PR #81, which in turn reconstructs PR #77's current delegation lineage.

Parent/child/policy/delegation revocation, stale policy generation, mixed intermediate generations, missing re-delegation permission, or expired lineage lease fails **before** the current-executor freshness bundle can qualify.

The separate grant freshness receipt remains required for the exact target executor grant as defense in depth and for a uniform direct/delegated current-authority model.

## 8. Direct grants receive real revocation semantics too

Direct executor grants have no delegation lineage, but they still require exact canonical grant freshness plus threshold and executor-designation freshness.

A direct grant cannot remain executable merely because its semantic expiry has not yet elapsed after institutional revocation.

## 9. Exact closed-set freshness only

The required current subjects are exactly:

1. canonical executor grant;
2. exact threshold authorization; and
3. exact lineage-bound executor authority/designation.

`qualify_current_freshness` must receive exactly those current receipts.

Missing subject, unexpected subject, conflicting current snapshots, revoked/superseded state, stale lease or malformed identity denies.

No timestamp/latest-record/DHT-order heuristic chooses a current generation.

## 10. Impossible current-state timing denies

A current state cannot become effective before its immutable semantic object exists:

- grant freshness may not predate grant issuance;
- threshold freshness may not predate threshold signature time; and
- executor freshness may not predate designation issuance.

Later effective times are permitted and represent subsequent activation/re-authorization generations.

## 11. Stable current authority identity

The final current-authority digest commits:

- canonical grant identity/profile;
- stable threshold-authorization identity/profile;
- PR #81 lineage-bound executor semantic identity/profile; and
- exact generation-bound freshness bundle/profile.

A semantic change or generation/state change changes current executor authority identity.

## 12. Re-verification does not mint new authority

Dynamic verifier timestamps, verifier references and reusable lease horizons are not hashed into stable current authority identity.

Re-verifying the same exact generations may refresh the dynamic evidence lease without changing the authority digest.

## 13. Lease is a reuse ceiling, not authority identity

`lease_until_ms` is the minimum of:

- PR #81's semantic/current-lineage lease; and
- the exact freshness bundle lease.

At or after that instant the authority must be re-qualified. A lease never overrides revocation or generation change.

## 14. Qualified current authority is not application data

`QualifiedCurrentExecutorAuthority` has private fields and intentionally does not implement `Deserialize`.

Application payloads cannot mint current executor authority.

## 15. Advisory systems remain outside authority

Phi, consciousness/model scores, reputation, stake, caller identity, proposal authorship, committee membership alone, or a caller-supplied `current=true` value cannot qualify any current executor authority.

## 16. Runtime follow-on remains fail-closed

A future `governance_executor_authority_verifier` must independently resolve the semantic inputs and current freshness receipts from authoritative sources, invoke this kernel, and project the resulting exact current-authority digest/profile into the lifecycle verifier ABI.

Missing provider, unavailable current-state source, stale generation, ambiguous state, revocation, profile mismatch, or decode failure must deny.

External effects remain disabled.
