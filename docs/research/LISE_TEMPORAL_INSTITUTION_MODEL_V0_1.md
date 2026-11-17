# LISE Temporal Institutional Control and Recovery Model v0.1

**Status:** finite-state research reference; not a standard, proof of a real institution, production control system or certification  
**Related:** IPB-S3 adversarial capture, IPB-S5 authority-preserving continuity, LARP v2  
**Implementation:** `lise-benchmarks/temporal-institution-reference/`

## 1. Research question

Can institutional capture and recovery failures be represented as temporal state transitions with reproducible counterexample traces, rather than assessed only through static role charts, vote counts or isolated authorization decisions?

The model explores a small, explicit transition system and reports:
- hard invariant violations;
- control-overlap conditions requiring independent challenge;
- unknown states caused by incomplete inventory;
- shortest traces under the declared transition alphabet;
- exploration depth, state count and truncation.

## 2. State abstraction

The current finite model represents:
- a single modeled coalition's control over appointment, agenda, treasury, credential, execution and oversight domains;
- whether the control inventory is complete;
- founder availability;
- recovery trigger verification and recovery quorum;
- old-session fencing;
- service mode;
- whether high-impact capability is enabled;
- current constitution epoch and credential epoch;
- whether old administrator privileges have been restored;
- emergency authority activation, expiry and model time;
- attempted oversight restoration plus typed owner-authority, independence, quorum and authority-epoch evidence.

This is intentionally a small abstraction. It does not yet represent arbitrary organization graphs, multiple overlapping coalitions, delegation chains, provider dependencies, rights-holder populations, jurisdiction-specific rules or bitemporal evidence.

## 3. Temporal properties

### P1 — Oversight independence

When the same modeled coalition controls oversight and at least one critical power domain (appointments, treasury, credentials or execution), the result is `ChallengeRequired`, not an automatic misconduct verdict.

Agenda control is tracked because it affects capture analysis, but the first finite oracle focuses the challenge trigger on appointment/resource/credential overlap.

### P2 — Degraded service cannot retain high-impact authority

When service mode is `DegradedReadOnly`:
- high-impact capability must be disabled;
- old sessions must be fenced.

A system may preserve essential read-only or life-safety service without restoring administrative or financial authority.

### P3 — High-impact recovery requires a complete recovery chain

After founder/operator unavailability, high-impact capability may be enabled only when:
- recovery trigger is verified;
- recovery quorum is met;
- old sessions are fenced;
- credential epoch has been revalidated against the current constitution epoch.

A quorum is not enough if stale credentials can still exercise old authority.

### P4 — No stale administrator resurrection

After a constitutional epoch change, restoration of an administrator credential from an older epoch is denied until credential authority is explicitly revalidated. Restoring data or service is not restoring authority.

### P5 — Emergency authority expires

An emergency capability with an expiry time cannot remain active at or beyond that time. A later action must not silently inherit the expired grant.

### P6 — Oversight restoration requires evidence

An attempt to restore independent oversight changes the control state only when owner-authority, independence and quorum evidence are all valid and bind the current constitutional epoch. Missing evidence is Indeterminate; invalid evidence or an epoch mismatch is Deny. A valid synthetic input is not a cryptographic signature or real-world authority verification.

### P7 — Emergency authority expiry

An active emergency grant with no declared expiry is Indeterminate. An expired grant that remains active is Deny.

### P8 — Open-world control inventory

An incomplete control inventory yields `Indeterminate` when no harder known violation is already present. Missing control edges are not proof of independence.

## 4. Transition semantics

The model exposes a fixed action alphabet, including both safe and unsafe operations:
- capture or restore control over a domain;
- mark the control inventory incomplete;
- mark founder unavailable;
- verify a recovery trigger;
- reach a recovery quorum;
- fence old sessions;
- restore degraded read-only service;
- restore an old administrator;
- change constitution epoch;
- revalidate credentials;
- enable or disable high-impact capability;
- grant, revoke and advance time past emergency-authority expiry.

Unsafe transitions are intentionally representable. If the model prevented them before evaluation, the checker could not produce a counterexample showing why they are unsafe.

## 5. Bounded exploration

`bounded_explore(max_depth, max_states)` performs deterministic breadth-first exploration from the declared initial state.

For each state, it evaluates the invariants and preserves the first trace for each distinct:
- `Deny` reason;
- `ChallengeRequired` reason;
- `Indeterminate` reason.

Because exploration is breadth-first, the first witness for a reason is shortest under the declared transition alphabet and unit transition cost.

Every report includes:
- maximum depth;
- visited state count;
- hard counterexample traces;
- challenge witnesses;
- indeterminate witnesses;
- whether the state budget truncated the search.

**No counterexample found within a bound is not proof of unbounded safety.** Any public result must publish the transition alphabet, initial state, bound, state budget, abstraction, excluded dimensions and counterexample traces.

## 6. Conformance suite

The initial trace suite contains 12 synthetic cases with a separate oracle. It includes:
- normal positive control;
- founder loss;
- safe and unsafe degraded recovery;
- stale administrator restoration;
- recovery with and without quorum / credential revalidation;
- emergency expiry;
- coalition overlap across appointment, treasury, credentials and oversight;
- attempted independent-oversight restoration without authority evidence (Indeterminate);
- incomplete control inventory.

The Rust integration test checks exact disposition and reason code. Unit tests separately check qualified restoration, invalid/stale evidence and emergency grants with missing expiry. The oracle is a development reference pending independent review.

## 7. Relationship to LISE and LARP

This model is complementary to, not a replacement for:
- IPB-S3 capture scenarios;
- IPB-S5 continuity and recovery contracts;
- LISE dependency/common-cause assurance;
- LARP contract rights, obligations, challenge and remedy semantics;
- owner-native Mycelix credential, appointment and recovery authority.

The next extensions should introduce:
1. multiple organizations and overlapping coalitions;
2. appointment, agenda, treasury and oversight control edges with temporal intervals;
3. explicit common-cause and provider dependencies;
4. local constitutional epoch and jurisdiction transitions;
5. rights/appeal/remedy reachability;
6. independent simple oracle and generated counterexample regression corpus;
7. mapping to owner-native S3/S5 facts without duplicating their authority state.

## 8. Acceptance and limits

The finite reference should pass formatting, Clippy, tests and checks at an exact commit. Its 12 trace fixtures must match their separate oracle. The bounded search must be deterministic, report truncation, and retain shortest counterexamples.

It does not establish:
- that a real coalition has captured an institution;
- that a person committed misconduct;
- that all possible institutional captures are represented;
- that all recovery schemes are safe;
- that the model is legally valid;
- that a real deployment is secure or legitimate.

Independent domain review, owner-qualified data and external replication are required before applying these properties to a live institution.
