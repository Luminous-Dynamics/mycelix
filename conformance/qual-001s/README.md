# QUAL-001S Semantic Qualification Corpus

This directory contains the implementation-independent semantic corpus for QUAL-001S.

The corpus is deliberately **not** a Rust crate and has no dependency on:

- Mycelix epistemic scoring/types;
- manufacturing COS semantics;
- Holochain runtime state;
- a particular verifier implementation;
- a particular GitHub Actions runner.

Its purpose is to make the qualification theorem executable by multiple independently implemented evaluators without allowing either evaluator to redefine the claim boundary.

## Three separate algebras

Every vector keeps these propositions distinct:

1. **Evidence disposition** — what is known about the supplied evidence.
2. **Execution/authority outcome** — what the trusted execution path actually established.
3. **Claim ceiling** — the explicit set of propositions that may be emitted from that evidence. This is a capability boundary, not a numeric confidence score or ordinal rank.

A stronger-looking value in one algebra never upgrades another algebra.

In particular:

`OBSERVED execution != VERIFIED qualification != ADOPTION != POST_ADOPTION_INDEPENDENCE`

## Corpus contract

Each vector specifies:

- exact proposition/source identity;
- epoch identity;
- requested claim set;
- admissible claim set;
- theorem and upstream evidence references;
- evidence disposition;
- authority/execution outcome;
- expected evaluator state;
- explicit nonclaims.

The corpus contains both availability positives and hostile/negative vectors.

## Metamorphic properties

An evaluator conforming to this corpus must preserve these transformations:

- removing a required dependency changes the result to `UNAVAILABLE` rather than `CONTRADICTED`;
- adding a qualifying constraint cannot widen the admitted claim set;
- changing any epoch member invalidates receipts bound to the prior epoch;
- substituting a source class is rejected rather than silently normalized;
- stale evidence cannot cross the temporal frontier into current authority;
- an authentic receipt cannot by itself elevate lifecycle state;
- duplicate evidence is not independent corroboration;
- unknown schema versions and duplicate keys fail closed.

These are corpus properties, not implementation suggestions.

## Independence requirement

QUAL-001S should consume these same vector bytes through at least two materially independent evaluators before S0/S1/S2 is considered qualified. The second evaluator must be reconstructed from this published corpus/contract rather than copied from the first evaluator's control flow.

## Canonical commitment bytes

S0 commitment bytes use the declared `RFC8785-JCS-IJSON-v1` profile. This removes property-order ambiguity and requires duplicate JSON property names to be rejected before canonicalization. QUAL-001S also constrains numeric identifiers to the exact-in-IEEE-754 integer range; stronger numeric domains must use strings rather than silently relying on floating-point JSON numbers. RFC 8785 is an informational RFC, so this repository treats the profile as an explicit protocol choice rather than an implicit property of JSON.

## S0 authority observation
The corpus includes an explicit availability case for `dispatch_result=ACCEPTED` with no observed run attribution. That state remains `UNAVAILABLE` for execution; API acceptance is not silently promoted to execution evidence.


`s0_authority_observation_v1.schema.json` separates five historically distinct propositions: request authentication, actor authorization, event authorization, workflow-source authentication, and run attribution. `dispatch_result=ACCEPTED` is an API observation only; it does not imply that a workflow run occurred. `run_attribution=OBSERVED` does not imply candidate conformance. The example is illustrative and is not evidence of an actual authorized dispatch.

## S0 dispatch envelope

The S0 trust-plane boundary has a separate machine-readable envelope schema at `s0_dispatch_envelope_v1.schema.json`. It binds exact repository/PR subject identity, current/proposed verifier commitments, registered successor profile, S0/S1 workflow identities and refs, a 256-bit dispatch nonce, timestamp, and `candidate_code_executed=false`.

The schema does **not** claim that the candidate ran, passed, was adopted, or became authoritative. The example in `s0_dispatch_envelope_v1.example.json` is illustrative only and uses placeholder commitments.

## Local evaluators

The two reference evaluators are deliberately small and independent:

```text
python3 conformance/qual-001s/evaluator_a.py
node conformance/qual-001s/evaluator_b.mjs
```

Evaluator A checks schema-shaped structure and invariant preservation. Evaluator B reconstructs semantic expectations from proposition/theorem vocabulary. Neither imports repository domain crates, executes candidate verifier code, or publishes authority.

A future qualified S1 implementation should consume the same corpus bytes without replacing either evaluator with candidate-authored expectations.

## Status

This corpus is a **design/evidence artifact only**. A committed corpus is not a QUAL-001S PASS, does not qualify a verifier, and does not authorize adoption.
