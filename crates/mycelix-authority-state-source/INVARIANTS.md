# Mycelix Authority State Source v0.1 — Normative Invariants

Status: **pure append-only state-source/projection kernel; not a Holochain authority source**

This crate supplies the source/projection half intentionally omitted by `mycelix-authority-freshness`.

The freshness kernel answers:

> Given an exact current snapshot, can it contribute to current authority?

This source kernel answers:

> Which exact generation/state follows from one complete, independently verified append-only transition lineage whose authoritative head is independently covered?

It does not persist transitions, verify institutional signatures itself, select a DHT record by recency, or enable effects.

## 1. Existence is not authority

`AuthorityStateTransition` is candidate data.

A transition becomes eligible for projection only through `VerifiedAuthorityStateTransition`, whose host verifier must independently establish:

- exact immutable transition record/content identity;
- exact record proof;
- exact institutional/rulebook authority reference;
- exact authority proof for this transition;
- exact stable authoritative state-source identity; and
- bounded verification evidence.

A non-empty `authority_ref`, proof reference, DHT entry, author identity, role, score, or caller assertion is not sufficient.

## 2. One exact immutable subject

Every transition and coverage receipt in one projection must bind the exact same `AuthoritySubjectRef` from PR #74:

- subject kind;
- namespace;
- subject ID; and
- exact identity digest/profile.

Same textual ID with different semantic bytes/profile is another subject and cannot share a state lineage.

## 3. Generation is causal authority state, not time

Generation begins at 1 and advances by exactly one transition.

v0.1 bounds a lineage to at most 256 transitions. Generation values outside that bound deny before arithmetic or platform-size conversion.

The projector never treats the numerically greatest arbitrary record as authoritative. It first proves a complete contiguous chain and then proves that chain reaches the independently covered source head.

## 4. Exact parent commitment

Every non-root transition commits:

- exact previous generation; and
- exact previous transition identity digest.

Therefore generation N+1 cannot be detached from or rebound onto different generation-N semantics.

Missing parent, skipped generation, wrong parent digest, orphan data, or competing same-generation transitions deny.

## 5. Root semantics

Generation 1 is exactly:

`Establish -> Active`

It has no previous generation/digest.

A revoked/superseded root, non-1 Establish transition, or non-Establish generation-1 record is invalid.

## 6. Closed transition state machine

v0.1 permits only:

- `Active -> Revoked` via `Revoke`;
- `Active -> Superseded` via `Supersede`; and
- `Revoked -> Active` via explicit `Reactivate` under a new generation.

`Superseded` is terminal in v0.1.

There is no implicit Active->Active generation bump. Re-verification of unchanged state does not create a generation.

Reactivation is explicit authority, not removal or mutation of the old revoked record.

## 7. Effective time validates causality but never selects a fork

Child `effective_at_ms` must be strictly greater than its parent.

Timestamps may invalidate an impossible chain.

They may not:

- select between competing generation records;
- choose a fork winner;
- substitute for generation; or
- resolve ambiguous source state.

A newer timestamp on a conflicting record still causes denial.

## 8. Stable logical authoritative source

Every verified transition and the mandatory coverage receipt in one lineage must come from the same `authoritative_source_ref`.

That ref denotes the stable logical authority-state source/trust domain, not necessarily one process instance or network node.

Operational replication, failover, or provider rotation must preserve the same verified logical source identity or use an explicit migration protocol; silently combining unrelated sources is forbidden.

## 9. Transition authority may evolve

Each transition independently binds its own institutional `authority_ref` and authority proof.

The authority that establishes generation 1 need not be byte-identical to a later rulebook-authorized revocation authority, provided the host independently verifies each exact transition authority.

The state-source kernel never infers transition authority from possession of the subject itself.

## 10. Mandatory authoritative coverage/head proof

A contiguous transition prefix is **not** evidence that it is current.

Without a separate completeness proof, an observer could provide generations 1-2 while withholding an existing generation-3 revocation and make the prefix appear current.

Every current or historical projection therefore requires `VerifiedAuthorityStateCoverage` binding:

- the exact subject;
- the exact logical authoritative source;
- exact current head generation;
- exact current head transition digest;
- exact current head status-record identity;
- coverage proof/provenance;
- verification time; and
- bounded reuse lease.

The reconstructed endpoint must match all three head fields exactly.

Coverage verification must occur no earlier than the effective time of the head it claims to cover.

A valid prefix whose endpoint does not equal the covered head denies with `CoverageHeadMismatch`.

Coverage is dynamic source evidence, not part of immutable transition identity.

## 11. Duplicate and fork semantics

Exact duplicate stable transitions for the same generation are harmless.

Different transition semantics claiming the same generation are a fork and deny.

Dynamic verification duplicates may only shrink the aggregate reusable lease; they never create another state-generation identity.

Coverage does not resolve a fork. It proves completeness only after the transition set itself is unambiguous.

## 12. Current projection is covered endpoint reconstruction

`project_current_authority_state`:

1. validates the exact coverage proof;
2. validates every transition receipt;
3. proves one subject and one logical source;
4. canonicalizes exact duplicate transitions by generation;
5. rejects generation forks;
6. requires generation 1;
7. proves every exact parent link and legal state transition;
8. proves the reconstructed endpoint equals the independently covered source head; and
9. only then selects that endpoint as current.

It does not select by input order, DHT order, author, score, highest generation seen locally, or timestamp.

## 13. Current snapshot interoperability

A successful current projection can produce a `VerifiedAuthorityFreshness` ABI receipt.

Its freshness snapshot is the exact covered current transition:

- same subject;
- exact generation/state;
- exact effective time; and
- exact immutable transition/status record ref.

The projection reference is derived from the stable projection identity. Dynamic verifier timestamps/lease horizons remain evidence, not state identity.

## 14. Historical as-of projection is separate

`project_authority_state_as_of` first proves the **full currently covered, unambiguous lineage**, then selects the state whose effective interval contains the requested historical instant.

This use of effective time is legitimate because one unique causal lineage through the authoritative covered head has already been proven.

Historical projections cannot be converted into live `VerifiedAuthorityFreshness` receipts.

Therefore:

- later revocation can deny new execution;
- historical audit can still establish that the subject was Active before revocation; and
- historical validity can never reactivate current authority.

A historical query cannot use a deliberately truncated old prefix to avoid later source evidence.

## 15. Projection identities are deterministic

Transition, lineage, and projection identities use explicit registered BLAKE3 framed profiles.

The lineage digest commits ordered transition identity digests after causal reconstruction.

The projection digest commits:

- exact subject identity;
- exact complete lineage identity through the covered head;
- selected freshness snapshot identity; and
- whether the projection is live-current or historical at one exact `as_of` time.

Input order cannot change these identities.

Coverage verification metadata is dynamic evidence and therefore does not rewrite historical transition identities.

## 16. Reusable verification can only shrink

The projection aggregates transition and coverage verification evidence conservatively:

- verification time is at least the newest required verification; and
- lease horizon is no later than the earliest required transition/coverage lease.

Lease expiry requires re-verification; it does not change generation by itself.

## 17. Current inactive state is still valid source truth

A current source projection may legitimately end in `Revoked` or `Superseded`.

The source kernel returns that truth rather than hiding it.

PR #74's current-authority qualifier is responsible for refusing inactive state as positive execution authority.

## 18. Historical invalidity is not ordinary revocation

Ordinary later revocation does not rewrite prior valid state.

If later investigation proves a supposedly verified transition or coverage proof was forged or invalid at the time, the host verifier must stop supplying it as verified evidence. Historical projection must then fail or reconstruct only from independently valid source evidence.

This distinction is required for accountable audit.

## 19. Advisory systems have no source authority

Phi, consciousness, reputation, stake, model recommendations, Guardian labels, caller identity, or timestamps cannot:

- create a generation;
- select a current transition;
- authorize revocation/reactivation;
- claim source completeness;
- resolve a fork; or
- restore superseded authority.

They may be evidence considered by a governed authority process, but only independently verified transition authority and source coverage enter this kernel.

## 20. Runtime follow-on remains fail-closed

A later Holochain/source adapter must:

- store transitions append-only;
- bind publisher/record provenance;
- independently verify institutional transition authority;
- independently establish source coverage/current-head truth;
- deny incomplete coverage, prefix omission, ambiguity and forks;
- expose one stable logical authoritative-source identity;
- use this projector rather than newest-record heuristics; and
- return bounded verification evidence.

Missing source, incomplete coverage, unavailable verifier, proof failure, fork, orphan, stale lease, malformed profile, or unknown transition class is denial.

External effects remain disabled.
