# Integral D6U — Explicitly Qualified Recursive/Fixpoint Derivations

Status: **ReferenceModelOnly**

## Purpose

D6U makes recursive semantic derivation explicit without making a cyclic DKG authoritative.

The governing distinction is:

```
cyclic DKG
!= recursive semantic authority

recursive convergence
!= truth
recursive convergence
!= currentness
recursive convergence
!= authorization
```

A recursive derivation is admitted only through an explicit, versioned profile and an exact bounded trace.

## Profile

`RecursiveDerivationProfileV1` freezes:

- rule ID: `recursive-fixpoint-v1`;
- profile/version;
- deterministic iteration order: `ascending-index-v1`;
- convergence rule: `state-equality-v1`;
- exact finite-carrier commitment;
- maximum iteration bound;
- maximum trace-item bound;
- ReferenceModelOnly claim ceiling;
- profile commitment excluding its own commitment field.

A recursive profile cannot be inferred from the presence of a cycle.

## Seed binding

`RecursiveSeedV1` commits:

- exact D6S projection commitment;
- exact semantic environment commitment;
- exact input commitment set;
- exact initial state commitment;
- seed commitment.

Changing any seed input changes the seed commitment.

## Trace semantics

Each `RecursiveIterationV1` contains:

- monotonically increasing iteration index;
- exact predecessor state commitment;
- exact successor state commitment;
- delta commitment;
- transition commitment.

The trace requires:

```
iteration[0].predecessor == initial_state
iteration[n].predecessor == iteration[n-1].state
indices == 0..N-1
N <= max_iterations
N <= max_trace_items
```

The trace is therefore replayable as a deterministic chain of state commitments without relying on delivery order or map iteration order.

## Convergence

A `Converged` result requires:

```
last.predecessor_state_commitment
==
last.state_commitment
```

This is an explicit state-equality fixpoint criterion.

A `NonConvergentWithinBound` result requires the trace to exhaust the declared iteration bound without reaching equality and preserves unresolved state.

Iteration-bound and resource-bound failures cannot be serialized as successful convergence.

## D6S integration

D6S now requires an exact `recursive_derivation_trace_commitment` whenever:

- the selected projection contains a semantic derivation/support cycle; and
- the derivation profile explicitly permits `recursive-fixpoint-v1`.

This closes the previous gap where a recursive profile could merely toggle a cycle gate without committing the actual bounded derivation trace.

The trace commitment is included in the D6S projection and receipt commitments.

## Non-amplification

A converged fixpoint does not raise the claim ceiling.

D6U receipts remain `ReferenceModelOnly` and cannot create:

- truth;
- authority;
- currentness;
- observer independence;
- authorization;
- actuation permission;
- capacity;
- consent.

Symthaea may propose recursive seeds, traces, or candidate convergence, but it cannot mint the qualified recursive profile or promote convergence to authority.

## Adversarial corpus

The source-level tests cover:

1. explicit/bounded recursive profile;
2. exact seed binding;
3. state-equality convergence;
4. deterministic iteration order;
5. predecessor-chain continuity;
6. iteration bound enforcement;
7. full-bound requirement for non-convergence;
8. missing seed binding;
9. profile mutation;
10. trace mutation;
11. explicit unresolved non-convergence;
12. converged receipt claim boundary;
13. exact D6S projection trace binding;
14. fixpoint does not create currentness.

## Claim ceiling

**ReferenceModelOnly.**

D6U does not establish truth, causality, production consensus, real-world convergence, legal authority, authorization, economic settlement, or actuation safety.
