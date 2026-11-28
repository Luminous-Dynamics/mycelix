# ITOPS outcome oracle reference fixtures v0.1

This directory contains a **public, synthetic known-answer corpus** for testing the ITOPS outcome adapter described in Mycelix #6846 and #6851.

It is not an agent task set, a hidden holdout, a production SRE harness, or evidence that any agent has been evaluated. Expected dispositions are intentionally public because these fixtures test evaluator semantics and mutation resistance.

## Files

- `itops-outcome-cases-v0.1.schema.json`: strict JSON Schema (Draft 2020-12).
- `itops-outcome-cases-v0.1.json`: 12 synthetic known-answer cases.

## Invariants exercised

1. A healthy/no-fault baseline can correctly require no mutation.
2. Accepted recovery requires independent persisted-state, functional and observation-horizon evidence.
3. Alert clearance is not fix-specific recovery.
4. Diagnosis without repair is not task success.
5. Tool acknowledgement does not override an independently observed unchanged state.
6. Partial action completion remains partial.
7. Provider acknowledgement alone does not establish external/service effect.
8. Two observers with a shared dependency do not count as independent corroboration.
9. A stale approval correctly denied is distinct from an unsafe mutation.
10. An out-of-scope effect is a hard-gate violation, even if the service remains healthy.
11. Recurrence before the declared horizon invalidates the recovery claim.
12. Unknown effect remains unknown; automatic retry is not inferred to be safe.

## How this composes

- Mycelix #6357 owns the verifier-created `VerifiedOutcomeLedger`; these fixtures must not introduce a parallel ledger or let caller-authored labels become authoritative.
- Mycelix #6140 owns evidence proposition, source relationship, expected-action coverage, temporal validity, shared dependency, partial and unknown semantics.
- Mycelix #5900 owns experiment/run identity, exact artifact references and independent reproduction lineage.
- Mycelix #4835 owns evaluator/oracle independence and the separation between qualification and authority.

The adapter mapping in #6851 is the ITOPS-specific layer. This directory is only its public reference oracle corpus.

## Required evaluator rules

- Enforce unique case IDs and exact expected-operation coverage in addition to JSON Schema validation.
- Every expected operation must have exactly one terminal record or explicit Unknown.
- Never derive accepted recovery from an agent claim, tool return, provider acknowledgement or alert state alone.
- Preserve Unknown, Partial, Unsupported, NotRun and Timeout as separate dispositions.
- Hard-gate violations are vetoes, not score penalties.
- Do not expose this public oracle fixture as the hidden answer key for a confirmatory agent tournament. Keep hidden challenge tasks and oracle custody separate.

## Competitive crosswalk

Use different public comparisons for different questions:

- ITBench-AA: root-cause entity diagnosis (59 tasks; its reported leading score is 47% on its published metric). This is not an end-to-end recovery score.
- SWEOps-Bench: verified recovery on 55 reproducible incidents (published configurations reported 14.5–52.7% overall recovery).
- PagerDuty's SRE Agent benchmark: 17 injected-fault scenarios; its report identifies one-trial-per-agent results as a statistical limitation.
- MLPerf Inference: inference/system latency, throughput and power under its own workload contracts; do not compare these directly with service recovery rates.

Sources and exact comparison conditions are tracked in #6846. All percentages are external reported values, not Luminous results.

## Claim ceiling

Static synthetic fixture and schema only. No Rust validator, executable environment, agent run, independent reproduction, production qualification, energy measurement or SOTA result is claimed by this corpus.
