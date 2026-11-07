# Pulse qualification evidence ledger v1

**Machine-readable ledger:** `pulse-qualification-evidence-ledger-v1.json`  
**Closed schema:** `pulse-qualification-evidence-ledger-v1.schema.json`  
**Research PR:** [#5771](https://github.com/Luminous-Dynamics/mycelix/pull/5771)  
**Ledger date:** 7 November 2026

## Purpose

Pulse now has several independent verification programmes. Their results are useful only when the project can say precisely:

- which exact source subject was evaluated;
- which executable, model, test, workflow, benchmark or receipt ran;
- whether that evidence is source inspection, a finite model, synthetic benchmark, cryptographic fixture, real runtime capture or multi-conductor observation;
- which steps actually completed;
- what the result does not prove.

This ledger is a **claim-control mechanism**, not another verifier. It does not upgrade weak evidence into stronger evidence. An entry from an older source SHA remains historical even when the same files are still present at a newer head.

## Claim ladder

| Level | What it establishes | What it does not establish |
|---|---|---|
| `SOURCE_PRESENT` | The implementation or test source exists at the recorded SHA | That it compiles or behaves correctly |
| `STATIC_OR_STRUCTURAL_CHECK` | Exact syntax/schema/source properties were checked | Runtime behavior or source authenticity |
| `UNIT_TEST` | Tested local functions/fixtures satisfy the encoded assertions | Conductor, DHT or multi-agent behavior |
| `BOUNDED_MODEL_CHECK` | The exact finite model preserves invariants and chosen mutants are detected | The real Holochain implementation, unbounded concurrency or liveness |
| `SYNTHETIC_BENCHMARK` | Performance on the declared synthetic workload and runner | Production throughput or cross-system superiority |
| `CRYPTOGRAPHIC_SYNTHETIC_EVIDENCE` | Signatures and policy bindings verify for declared synthetic fixtures | Real conductor truth, custody, SCITT receipt authenticity unless explicitly implemented |
| `RUNTIME_CAPTURE` | Exact instrumented runtime observation under declared conditions | Multi-conductor acceptance or broader topology |
| `MULTI_CONDUCTOR_RUNTIME` | Actual peer/validator/observer behavior for the recorded topology and faults | General Byzantine correctness |
| `PRODUCTION_QUALIFICATION` | A reviewed release gate passed for an exact deployment profile | Universal absence of defects |

These levels are not a simple quality score. For example, a bounded model check can provide stronger evidence about a named state-machine invariant than a unit test, but it cannot establish that the production Holochain conductor implements the modeled state machine.

## Status vocabulary

- `PASS_FOR_EXACT_SUBJECT`: the recorded exact subject completed the stated gate.
- `HISTORICAL_PASS`: the pass is real but belongs to an earlier source SHA.
- `SOURCE_ONLY_UNQUALIFIED`: source is present, but the required execution evidence is absent.
- `PENDING_NO_JOB_EVIDENCE`: a workflow was observed pending and no executable job evidence was returned.
- `ACTION_REQUIRED_NO_JOB_EVIDENCE`: GitHub reported action_required; do not infer that checkout or tests ran.
- `INCOMPLETE_BY_DESIGN`: a verifier intentionally abstains because a required trust or evidence layer is not implemented.
- `SELECTED_NOT_QUALIFIED`: a target/profile has been selected, but its qualification gates have not passed.
- `OPEN_RUNTIME_GATE`: abstract/source evidence exists, while actual runtime acceptance remains open.

A timeout, missing job, skipped step, empty job list, or queued workflow is not a failed source test and is not a pass. Record the distinction.

## Current evidence summary

The machine-readable ledger now records twelve areas:

1. **TLA+ assurance suite — exact PR head `acd9ed7e6d4779e17b936ab5589d58ebfb7bdc39`.** TLC 1.8.0 run #346 completed nine safe finite models and caught seventeen named negative-control mutants. The restore-specific receipt is `PulseChainRestore-model-check-2026-11-07.md`. All results remain bounded abstract evidence.
2. **Authorization corpus.** The current JSON contains 28 unique scenarios. This is a synthetic scenario inventory, not 28 runtime tests.
3. **Multi-conductor baseline.** Run #81 measures a two-conductor, no-cross-edge synthetic history profile. Its 1M event sample is one capacity probe, not a percentile population.
4. **Borrowed parser benchmark.** Run #166 reports a 3.2–3.4x parse/shape-validation speedup. It does not demonstrate a whole-auditor speedup; production still materializes the owned semantic history.
5. **Signed evidence verifier.** Run #90 passed 26 synthetic tests at its recorded earlier head. SCITT COSE/receipt/VDS verification, production key custody and independent root-policy implementation remain gaps.
6. **Pulse 0.7.0 canary.** Current PR #4891 head reported `action_required` with no job records. Compilation, WASM and Sweettest are not established for that head.
7. **Mailbox two-conductor path.** Source/test changes exist, but the exact-head workflow had no job evidence at the last retrieved observation.
8. **Holochain 0.7.1 target.** Upstream target selected, Mycelix runtime not qualified.
9. **Public chain restore.** Bounded restore model passed; real `restore_from_dht` qualification remains open under #5905.
10. **SCITT transparency.** Structural/digest checks only; cryptographic receipt and VDS proofs remain incomplete.
11. **Per-cell capability isolation — exact model subject `d206c74670475e2e8fcd9f720e0f804af7469733`.** TLC run #356 completed the safe two-cell grant model with 66,674 generated / 12,687 distinct states and caught three named mutants: cross-cell borrowing, function-scope widening and collateral revocation. Artifact 13585841413. This is bounded abstract evidence only; it does not replace the two-conductor host-capability Sweettest.
12. **Cell-scope model repeat on latest branch subject `acca54020e9f49de0bc8aeea39325cc848fdb9dd`.** Run #359 reran the same 66,674 / 12,687-state safe model and caught the same three scope mutants; artifact 13585459439. This is reproducibility evidence for the same bounded model, not a larger state space or conductor runtime test.

The JSON carries source SHAs, workflow/run IDs, artifacts, results and claim ceilings. Never cite a historical pass as evidence that the current branch head passed.

## Required maintenance rules

1. Update the ledger whenever a qualification result, source head or claim ceiling changes.
2. Bind each result to the exact source SHA and tool/build/config hashes; do not rely on PR number or branch name alone.
3. Keep raw logs, benchmark outputs, counterexamples and receipts available by digest.
4. Record the workload, topology, faults, sample class and repetition count for benchmarks.
5. Keep single capacity probes distinct from independent percentile populations.
6. A benchmark speedup cannot override any safety mutation.
7. A valid signature proves who signed the covered bytes under the declared key/profile; it does not prove the statement is true.
8. A DHT projection, signed signal, checkpoint or receipt never replaces the conductor's current host capability decision or domain-resource authorization.
9. Re-run affected exact-head gates after rebasing/merging main or changing source, workflow, dependency lock or model configuration.
10. The ledger itself must be reviewed. An automated parser can validate its schema; it cannot independently authenticate every referenced GitHub artifact.

## Immediate release gates

- Reconcile the research branch with current `main` before ready-for-review. The latest comparison on 7 November 2026, from research branch subject `acca54020e9f49de0bc8aeea39325cc848fdb9dd` to `main` head `6a0d072d3b65f4a2cf08af3ae71e541788e33ae9`, shows 431 commits ahead and 8 behind, with merge base `9e12dd46bdd8371e8ed9626a17bc06f21c141a52`. The PR remains draft and has a broad 100+ file diff. Reconcile and review scope before merge, then rerun the full suite on the integrated exact head.
- Select Holochain 0.7.1 as the forward-development target; retain 0.7.0 as a historical regression canary.
- Complete the exact 0.7.1 Pulse source/build/WASM/fresh-conductor/multi-agent and independent-client chain.
- Demonstrate mailbox authorization with a malicious coordinator and receiving production integrity WASM.
- Exercise real public chain restoration on multiple 0.7.1 conductors.
- Promote the borrowed parser work into a semantically borrowed/streaming audit only after report equality and all existing mutation tests pass.
- Complete actual SCITT receipt and VDS cryptographic verification before assigning transparency-authenticated status.

**Global claim ceiling:** the current programme has strong bounded-model and synthetic-verifier evidence, plus historical synthetic benchmarks. It does not yet establish production-ready Holochain 0.7.1 behavior, general distributed-system linearizability, Byzantine consensus, universal partition safety, or performance superiority over industry systems.
