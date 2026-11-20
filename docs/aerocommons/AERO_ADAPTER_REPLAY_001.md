# AeroCommons adapter replay benchmark — v1

Status: synthetic pairwise evaluator; no real AP242/QIF converter runs are included  
Tracker: https://github.com/Luminous-Dynamics/mycelix/issues/6358  
Implementation: https://github.com/Luminous-Dynamics/mycelix/pull/ (draft PR will be linked after creation)  
Parent receipt contract: #6354 / PR #6355

## Why replay is its own benchmark

A content-verified receipt describes one declared adapter run. It cannot, by itself, establish that a repeated invocation reproduces the same output or that two outputs with different bytes preserve the same engineering meaning.

The benchmark distinguishes four non-equivalent properties:

1. **Byte reproducibility** — the same exact execution context produces byte-identical raw outputs and normalized assertion output.
2. **Normalized-output byte stability** — normalized assertion bytes match even though one or more raw artifacts differ, for example because a serializer records a timestamp.
3. **Semantic reproducibility** — normalized assertions preserve engineering meaning under the separately qualified semantic evaluator and independent oracle (#6349). This crate does not determine it.
4. **Differential interoperability** — independent adapter implementations agree on the frozen assertion corpus. A different executable is not a repeat execution of the same adapter and must not be misreported as nondeterminism.

## Comparison protocol

`compare_verified_runs` first calls the #6354 verifier on both receipts and their complete supplied payload sets. It therefore checks:
- receipt canonical digest;
- exact executable, argument vector and environment-profile bytes;
- exact declared inputs and outputs;
- configuration, role/format and rights declaration consistency;
- declared/observed egress accounting;
- the exact payload set, with no missing, unexpected or duplicated references.

Then it compares the run contexts. Context equality is order-invariant for input/output arrays and egress lists, while preserving each artifact's role and exact content metadata. A changed adapter executable, source, argument vector, environment profile, configuration, rights declaration or network policy makes the pair `INCOMPARABLE_CONTEXT`; it is not output nondeterminism.

Each successful, context-equivalent pair is reported as one of:
- `EXACT_BYTE_REPLAY`: all raw and normalized output bytes match;
- `ASSERTIONS_BYTE_STABLE_RAW_OUTPUT_DRIFT`: normalized assertion bytes match but raw output bytes differ;
- `NORMALIZED_ASSERTIONS_BYTE_DRIFT`: normalized assertion bytes differ and require #6349's evaluator and independent adjudication.

If either run fails or times out, the pair is `RUN_NOT_ELIGIBLE`. The offered pair remains in the summary denominator.

## Canonical limits

This is a pairwise verifier over two supplied receipts, not a runner or sandbox. It does not:
- execute an adapter or independently attest that a process ran;
- instrument sockets or enforce the receipt's network policy;
- authenticate the adapter publisher or signer;
- interpret the engineering semantics inside normalized assertions;
- compare two different toolchains as if they were identical builds;
- treat byte stability as physical truth, conformance or acceptance.

For allowlisted network runs, the receipt verifier can only check declared/observed egress metadata. Real deterministic replay should start with network-disabled execution or content-address every remote dependency and response.

## Frozen adversarial scenario panel

The panel manifest at `fixtures/aero-adapter-replay/replay-panel-v1.json` names the initial controls. The Rust tests construct exact synthetic receipts and byte payloads, then pass them through the same public verification entry point.

Required controls include:
- exact replay despite input/output record permutation;
- raw QIF drift with unchanged normalized assertions;
- normalized-assertion drift with unchanged raw output;
- changed source, adapter executable, arguments, environment, configuration, rights or network policy;
- failed and timed-out executions retained as ineligible;
- duplicate run identity and tampered receipt digest;
- missing/extra/duplicate payloads.

Positive controls are mandatory: exact replay and list-order permutation must both pass. A benchmark that rejects every pair is invalid.

## Industry comparison and baseline selection

CAx-IF conducts two vendor interoperability test rounds per year. The Summer 2026 round used Test Suite 1.2. The published downstream-use Test Cases 7 and 8 provide paired STEP/QIF artifacts with trace information. They should anchor the first rights-cleared real-data campaign.

NIST STEP File Analyzer and Viewer 5.41 supports STEP AP242 and reports semantic/graphic PMI and recommended-practice findings. Use it as an independent validator within its documented scope, not as a tool to be displaced by this replay crate.

The comparison matrix should be:
- CAx-IF Test Cases 7/8 with exact practice/version and source hashes;
- NIST SFA reports and tool version;
- at least two runs of each identical adapter version/configuration/input set;
- at least two independently implemented adapters where rights and semantic coverage allow;
- #6349 assertion evaluation and a sealed independent oracle for every semantic disagreement.

## Metrics and prospective targets

These are software and benchmark targets, not achieved results or regulatory acceptance criteria:

| Metric | Proposed target |
|---|---:|
| Agreement with an independent pairwise reference model | 100% frozen corpus |
| False exact replay when execution context/output differs | 0 |
| False incomparable result from harmless list-order changes | 0 |
| Failed/timeout preservation in offered-run accounting | 100% |
| Distinction of byte stability from semantic adjudication | 100% |
| Semantic-pass claims emitted by replay crate | 0 |

Report exact-byte replay rate, normalized-assertion byte-stability rate, normalized-output drift rate, context-incomparable rate, failed/timeout rate, payload bytes processed and verifier p50/p95 latency. Keep each denominator explicit.

A real SOTA claim requires a sealed rights-cleared corpus, independent reference implementation, repeated execution per adapter/configuration, at least two distinct adapter implementations, third-party adjudication and matched compute/operator budgets. Synthetic tests are not performance evidence.

## Integration sequence

1. Qualify the one-run content-bound receipt (#6354).
2. Qualify this pairwise replay classifier.
3. Build actual adapter invocations with sandbox/egress instrumentation and runtime attestation.
4. Replay the CAx-IF/NIST corpus with at least two executions per adapter.
5. Pass normalized assertion sets to #6349 and record semantic outcomes separately.
6. Link qualified unit/configuration receipts through #6347 and change-impact closure through #6351.

Symtropy #2890 continues to own decision-aware experiment selection and matched-budget VOI; this benchmark does not duplicate an experiment optimizer.

## Claim ceiling

A successful `EXACT_BYTE_REPLAY` establishes only that two content-verified receipts have equal declared contexts and equal output bytes under this v1 comparison. It does not prove adapter authenticity, trusted execution, semantic preservation, measurement accuracy, manufacturing capability, product acceptance, certification or airworthiness.
