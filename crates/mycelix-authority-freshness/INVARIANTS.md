# Mycelix Authority Freshness v0.1

Status: **normative pure semantic kernel**

This crate separates immutable authority identity from current revocation/freshness state.

It does not create grants, policies, signatures, delegations, executor designations, effect permissions, or revocations. A runtime authority source must independently establish the current state facts supplied to this kernel.

## Three different questions

A secure authority system must not collapse these questions:

1. **What authority object existed?** — immutable object/content identity.
2. **Was it valid at a historical instant?** — historical verification/projection.
3. **Is it still current now?** — current generation/state + fresh verification lease.

Revocation changes question 3 without rewriting answers to questions 1 or 2.

## Subject identity

`AuthoritySubjectRef` binds:

- one closed subject kind;
- one namespace;
- one explicit subject ID; and
- one exact non-zero content digest + canonical digest profile.

A bare string ID is never sufficient current-authority identity. Same-ID rebinding to changed bytes/profile produces a different subject identity.

v0.1 subject kinds are:

- institutional authority grant;
- signing policy;
- threshold authorization;
- executor designation;
- effect-safety policy; and
- delegation.

Adding another authority class requires an explicit protocol/schema evolution; unknown classes do not silently fall back to generic authority.

## Stable snapshot identity

`AuthorityFreshnessSnapshot` binds:

- exact immutable subject;
- non-zero generation;
- state (`Active`, `Revoked`, `Superseded`);
- state-effective timestamp; and
- immutable status-record/content reference.

Its canonical digest excludes:

- verifier invocation time;
- verification reference;
- freshness lease horizon; and
- transport/DHT arrival order.

Therefore re-verifying unchanged current state later preserves snapshot and bundle identity.

## Generation semantics

Generation is authority state, not a clock.

A runtime provider must maintain/verify monotonic generation semantics for one subject. This pure kernel does **not** infer current state by selecting the largest generation from arbitrary history.

The runtime must resolve one current snapshot per required subject before calling this kernel.

A generation/state/status-record change changes the freshness identity immediately.

## Fail-closed current-set qualification

`qualify_current_freshness` requires an exact closed set of subjects.

- duplicate requirements deny;
- missing required subjects deny;
- unexpected subjects deny;
- exact duplicate receipts for the same stable snapshot are harmless;
- different snapshots for the same subject are ambiguous and deny;
- a `Revoked` or `Superseded` subject cannot contribute to positive current authority;
- expired verification leases deny;
- future/inconsistent verification times deny; and
- input order never selects or changes the result.

The kernel deliberately does not choose by timestamp, DHT arrival, record recency, author, score, reputation or model output.

## Bundle identity

A successful result returns a deterministic `freshness_digest` over canonical subject order and each exact stable snapshot identity.

This digest is suitable for incorporation into higher-level runtime authority references.

Examples:

- threshold runtime authority can bind current signing-policy / committee-epoch / signature-evidence generations;
- executor runtime authority can bind current grant / delegation / executor-designation generations;
- effect-safety authority can bind the generation of the qualified adapter/policy state.

If any required generation changes, the higher-level authority reference/domain must change.

## Dynamic freshness evidence

`VerifiedAuthorityFreshness` adds dynamic evidence:

- authoritative source reference;
- verification reference;
- verification timestamp; and
- bounded lease horizon.

These fields prove the snapshot is sufficiently fresh **now** but are excluded from authority identity.

A later re-verification may extend/replace this dynamic evidence without invalidating an already-issued designation or historical lifecycle record, provided the stable generation/state identity is unchanged.

## Current vs historical verification

Current execution must require current positive freshness.

Historical audit/replay must instead resolve the generation/state that was authoritative *as of the historical event time*.

A later revocation therefore blocks new execution while preserving the ability to prove that an older action was authorized when it occurred.

## No advisory authority

Phi, consciousness, reputation, stake, model confidence, planner recommendations and other advisory signals are not inputs to this kernel. They cannot select a generation, revoke authority, restore authority, or choose between conflicting current-state records.

## Delegation follow-on

This crate gives delegation its own revocable subject kind but does not prove delegation attenuation.

Issue #73 separately requires an exact delegation-lineage kernel/provider binding child grant to parent grant, holder/delegate, institution, rulebook, capabilities, lifetime, parent generation and cycle/depth constraints.
