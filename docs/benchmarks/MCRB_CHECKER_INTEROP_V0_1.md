# MCRB Checker Interoperability v0.1

**Status:** proposed, fixtures not executed  
**Owner:** [#6415](https://github.com/Luminous-Dynamics/mycelix/issues/6415)  
**Related compound-fault corpus:** [#6407](https://github.com/Luminous-Dynamics/mycelix/issues/6407), [PR #6408](https://github.com/Luminous-Dynamics/mycelix/pull/6408)

## Purpose

MCRB should not invent a universal consistency checker. It should preserve one evidence-qualified history, route that history to independent checkers only when their semantics apply, and retain the exact checker/specification/output identities.

Machine-readable known-answer fixtures are in:

- `MCRB_CHECKER_INTEROP_V0_1.json`
- `MCRB_CHECKER_INTEROP_V0_1.schema.json`

They contain eight synthetic cases covering a sequential one-shot consume, concurrent double consumption, a lost acknowledgement, exact idempotent retry, changed-fingerprint reuse, independent conductors with no causal edge, transactional write skew, and a missing cross-conductor validator join.

All fixtures are `proposed-not-executed`. They are test inputs and expected dispositions, not test results.

## Checker selection and claim boundaries

| Method | What it checks well | MCRB use | What it does not establish |
|---|---|---|---|
| Jepsen | Fault-injected operation histories against a named consistency model | Export invoke/ok/fail/info histories with a nemesis/fault timeline | All privacy, authority, observer-independence or recovery guarantees |
| Elle | Transactional anomalies inferred from transaction histories and dependency cycles | Use for supported transactional profiles, e.g. a write-skew cycle | Generic Holochain DHT convergence or external-effect truth |
| Porcupine | Linearizability against a declared sequential object specification | Check #6193 one-shot capability consumption/effect admission | Unspecified consistency models or unresolved effects converted to invented return values |
| FoundationDB Simulation | Deterministic simulation integrated with the production execution model, combined with live and hardware testing | Long-term design reference for an integrated simulation backend | A claim that MCRB's deterministic schedule replay is equivalent to an integrated deterministic simulator |
| Antithesis | Deterministic hostile full-stack execution and reproducible failures | Long-term design reference for dependency/client/checker fault exploration | A claim that a finite hand-authored schedule provides comparable state-space coverage |

References:
- Jepsen design and history checking: https://github.com/jepsen-io/jepsen and https://jepsen-io.github.io/jepsen/jepsen.core.html
- Elle consistency checker and its stated completeness caveats: https://github.com/jepsen-io/elle
- Porcupine linearizability checker: https://github.com/anishathalye/porcupine
- FoundationDB simulation/testing: https://github.com/apple/foundationdb/blob/main/documentation/sphinx/source/testing.rst
- Antithesis execution and deterministic reproduction: https://antithesis.com/docs/introduction/welcome/ and https://antithesis.com/docs/introduction/how_antithesis_works/

## Semantic rules

### Operation history

Map the operation phases to Jepsen's established vocabulary:

- `invoke`: the client begins an operation.
- `ok`: a successful completion was observed.
- `fail`: a definite failure was observed before an effect could occur.
- `info`: the outcome may have occurred, but the client cannot establish whether it did.

An `info` event is not a failure. It remains Unknown until qualified reconciliation. A timeout after dispatch, a lost acknowledgement, or a crash after provider acceptance must not be rewritten as a definite non-effect.

Fault injections are separate nemesis/timeline events. They are never client terminal outcomes.

### Causal history

For multi-conductor observations, reuse #5843's Pulse v2 partial-order schema/verifier where its event semantics apply. Preserve each conductor's local sequence and clock epoch. Cross-conductor happens-before requires an explicit evidence-backed edge. No scalar local sequence, wall-clock timestamp or topological ordering may invent a missing edge.

### Transactional history

Elle is eligible only when the input identifies transactions, operations, datatypes and a supported consistency profile. A cycle result is a counterexample for that profile, not a generic application failure.

### Linearizability history

Porcupine is eligible only with an exact sequential object specification. Bind both the specification digest and input-history digest. An unknown outcome must remain unknown; do not fabricate a return value to force a linearizability result.

## Result composition

Checker outputs are evidence inputs, not authorization to publish QOPS. Every output must bind the exact history, source/tree, scenario and operation contracts, runtime/topology, schedule/seed, checker binary, consistency profile or sequential specification, and output digest.

If semantically comparable checkers disagree, retain the disagreement as Indeterminate and investigate it. Do not majority-vote.

QOPS still requires the verified offer ledger, authenticated terminal outcome ledger, valid timing evidence, correctness gates, observer provenance and witness policy. Synthetic fixtures cannot yield eligible QOPS.

## Required next implementation

1. Add deterministic exporter adapters from the verified MCRB evidence model.
2. Reuse #5843's partial-order audit for compatible Holochain conductor evidence.
3. Connect #6193's one-shot capability model to a Porcupine-compatible sequential specification and independent history.
4. Add an Elle adapter only for transactional operation contracts.
5. Add Jepsen history export and fault/nemesis timeline binding.
6. Run the eight known-answer fixtures through exact checker binaries and independent oracles.
7. Measure checker throughput, peak memory, ambiguity, evidence coverage and adapter overhead separately.

## Claim ceiling

The fixture corpus is synthetic and has not been executed. It does not demonstrate a checker adapter, MCRB runtime qualification, linearizability of a deployed Holochain application, Byzantine fault tolerance, DHT convergence or SOTA performance.