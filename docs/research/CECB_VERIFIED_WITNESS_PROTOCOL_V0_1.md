# CECB Verified Witness Protocol v0.1

**Status:** protocol proposal and schema fixtures. This is not yet a production candidate runner, a validated theorem prover, or evidence that a discovery method is superior.

Related implementation work:
- [CECB known-truth scorer and discovery PR #5515](https://github.com/Luminous-Dynamics/mycelix/pull/5515)
- [Matched-budget tournament issue #5517](https://github.com/Luminous-Dynamics/mycelix/issues/5517)
- [CECB-WITNESS-001 issue #5519](https://github.com/Luminous-Dynamics/mycelix/issues/5519)

## Principle: a counterexample is an independently checked object

A candidate does not get credit for saying that it found a counterexample. It must submit a concrete, in-domain assignment. The trusted verifier independently replays that assignment against the candidate semantics and the reference semantics. Only a feasible assignment that reproduces a disagreement can receive the outcome `valid_counterexample`.

The protocol reserves three outcomes with deliberately different meanings:

| Outcome | Meaning | Must never be interpreted as |
|---|---|---|
| `valid_counterexample` | The witness is feasible and an independent replay reproduces a candidate/reference mismatch. | Proof that a proposed repair is correct. |
| `no_counterexample_found_within_budget` | Search ended without a valid witness before its budget was exhausted or terminated. | Evidence that the claim is correct, or a proof of no counterexample. |
| `verified_no_counterexample_in_bounded_domain` | The complete declared finite domain was checked, or an independently validated proof artifact establishes the result. | A general proof outside the declared domain or proof scope. |

The difference between “not found” and “cannot exist in the checked domain” is a core correctness property. It must survive serialization, logging, dashboards, aggregate metrics, and any downstream agent that consumes the receipt.

## Receipt contract

The JSON Schema is at [CECB_VERIFIED_WITNESS_RECEIPT_V0_1.schema.json](schemas/CECB_VERIFIED_WITNESS_RECEIPT_V0_1.schema.json). The three checked-in fixtures cover all three outcomes.

Every receipt binds:
- the exact source checkout SHA;
- digests of the candidate semantics, reference semantics, and declared domain;
- verifier and candidate method versions;
- random seed (or explicit null when no randomness is used);
- the maximum and actual label-query counts;
- a query trace containing request/response digests rather than raw label values;
- the outcome and structured verification evidence;
- optional measured runtime and peak memory.

The witness, when present, contains a concrete feature assignment, feasibility result, candidate prediction, reference prediction, and witness digest. The schema rejects an accepted counterexample whose predictions agree or whose feasibility is false.

SHA-256 fields provide content identifiers; they do **not** authenticate who produced the receipt. If receipts cross trust boundaries, add a separate signed envelope with key identity, signature algorithm, and a documented trust policy. A claim or domain digest also does not prove that its hashed artifact is semantically complete: reviewers must inspect and version the underlying specification.

## Verification rules

### Valid counterexample

The trusted verifier must establish all of the following:
1. The assignment is inside the declared domain.
2. All hard feasibility constraints hold.
3. The candidate and reference semantics are executed independently on the same assignment.
4. Their relevant outcomes disagree.
5. The witness and claim digests match the run's pinned artifacts.

A candidate-provided “valid” flag, explanation, confidence score, or self-reported label is not evidence. The receipt should contain no latent/reference label ledger or holdout labels beyond the witness that is intentionally published after acceptance.

### No counterexample found within budget

The verifier must record the actual label-query count from the trusted oracle ledger. The receipt must not include a witness and must not claim the whole domain was checked. Stopping due to query budget, time budget, memory, crash, or an unsupported domain belongs here (with the concrete termination reason recorded in the runner's extended report when available), not in the proof outcome.

### Verified no counterexample in bounded domain

This outcome has only two allowed routes:
- exhaustive checking of the complete, finite discrete domain; or
- a separately validated proof artifact, referenced by digest.

For exhaustive checking, a semantic validator must check that the declared domain kind is finite-discrete and that the verifier's checked-domain size equals the declared domain size. A schema-valid receipt whose counts disagree is still invalid. For proof mode, the proof verifier and proof artifact format must be versioned independently; a filename or natural-language explanation is not a proof.

## Candidate boundary and accounting

An untrusted candidate receives only the task specification, candidate-visible inputs, and a budgeted query capability. The trusted harness owns reference labels, feasibility checks that reveal hidden semantics, domain completion status, and the evaluator. Never pass the evaluator-bearing object or full fixture into candidate code.

The query trace must come from the harness, not candidate metadata. The harness must count attempted calls, including repeated or invalid requests. Wall time, peak memory, crashes, and external simulator calls must be measured outside candidate-reported output. A schema can reject malformed structure, but cross-field invariants—such as query-trace length equalling actual queries used—require a semantic validator.

The sample fixtures in this directory use fixed, syntactically valid placeholder hashes for schema tests. They are **not** receipts from executed experiments, and their hashes are not claims about an actual source build. CI validates schema and semantic invariants using these fixtures and deliberate invalid mutations.

## Paired falsification and non-falsification controls

A falsification benchmark must contain both falsifiable and true-claim controls. “Always emit a counterexample” must fail on true claims; “never emit a counterexample” must fail on falsifiable claims. For a finite known-answer task, report at least:
- valid-counterexample precision and recall;
- invalid-witness and infeasibility rates;
- discovery success versus query budget;
- true-claim rejection rate;
- duplicate-witness rate and witness minimality when a declared shrinker exists;
- repair success and regression rate after the original witness and all retained regression witnesses are replayed.

Report these separately by claim template, domain shape, structural model family, seed, and budget. Do not collapse witness validity, search efficiency, true-claim recognition, and repair quality into one weighted score. Publish uncertainty across independent families and seeds; public fixtures are software tests, not blind generalization evidence.

## CI qualification

The dedicated workflow validates the Draft 2020-12 schema, accepts all three well-formed fixtures, and rejects mutations that:
- make candidate and reference predictions agree in an alleged counterexample;
- attach a witness to a no-witness outcome;
- claim exhaustive completion after checking fewer points than the declared finite domain;
- exceed the recorded query budget.

These checks validate the receipt contract, not any particular counterexample-search algorithm. The implementation and the benchmark still need independent review before making claims about SOTA or real-world validity.
