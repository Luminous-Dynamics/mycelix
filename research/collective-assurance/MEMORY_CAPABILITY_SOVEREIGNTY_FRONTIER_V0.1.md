# Memory Capability–Sovereignty Frontier (MCSF) v0.1

**Status:** initial Rust matched-run manifest and non-compensatory scorecard qualifier. The synthetic fixture is a contract smoke test, not an experiment or a performance claim.

## Paradigm

MCSF measures the capability–sovereignty frontier: how much task utility a memory architecture can provide while preserving boundaries, revocation, causal traceability and resource accountability. The result is a vector, not a weighted composite. A retrieval win cannot compensate for unauthorized disclosure; a complete privacy result cannot fill missing task utility or cost evidence.

## Matched controls

The suite manifest binds the exact task set, paired-world scenario manifest, evaluator, answer model, prompt, tools, hardware, budgets, seed, Top K and architecture inventory. Each run carries the canonical digest of those controls. A run using a different task set, evaluator, answer model, prompt, toolset, hardware or budget cannot be compared as matched.

The control digest is order-independent for task IDs and architecture inventory. It uses a domain-separated SHA-256 over the full control manifest.

## Architecture conditions

The initial enum supports NoMemory, BM25, DenseVector, HDC, HDC+CFC, hybrid sparse+dense+HDC+CFC, graph/relational memory, governed Mycelix and coordinated Mycelix. The first pilot should include at least NoMemory, one conventional retriever and one HDC/CfC system.

Every run records exact source commit/artifact digest, memory representation, indexing/update policy and architecture configuration. NativeUnrestricted is an explicit policy class, not an official AML division. An OfficialDivisionCompatible run requires a separate policy qualification receipt.

## Causal exposure composition

MCSF does not implement a second generation-input signature, replay guard, or evidence ledger. It consumes Mycelix [#6700 — signed generation-input exposure receipts](https://github.com/Luminous-Dynamics/mycelix/pull/6700), which binds generation/request/responder/purpose/model/prompt/nonce, exact payload digest and ordered evidence inventory, and reuses the existing #6619 signature-verifier and replay-guard interfaces.

The MCSF task observations then measure a different downstream distinction:

- **retrieved:** the memory candidate was returned by the retrieval system;
- **consumed:** the exact item appears in the independently observed model-input witness;
- **influential:** a paired counterfactual replay attributes a measurable decision/task-output difference to the item.

The ordering invariant is `retrieved >= consumed >= influential`. #6700's receipt qualifies what was present at the generation-input boundary; it does not by itself prove attention, causal influence, or answer correctness. Influence requires a separate, preregistered counterfactual replay report. MCSF references both receipts and replay artifacts rather than promoting either into a truth oracle.

## Non-compensatory output vector

Each architecture scorecard separately reports:
- native task utility and paired delta against NoMemory on matched task IDs;
- task-family utility means and paired family deltas, so a strong fact-recall score cannot conceal a temporal, governance, personalization or streaming regression;
- deterministic paired-bootstrap 95% percentile intervals for overall and family-level deltas when at least two paired tasks are available. The resampling seed is bound to the matched-control digest and architecture/family; smaller strata return no interval rather than a fabricated estimate;
- authorized-task success count;
- unauthorized content disclosures and forbidden metadata inferences;
- privacy-observer evidence completeness;
- revocation expected/reconciled/unknown descendants;
- retrieved, model-input-consumed and decision-influential memory counts;
- causal witness completeness;
- input/output tokens, memory storage, index-write/search/wall time, optional energy and provider cost;
- role-enforcement evidence for coordinated Mycelix runs;
- policy-class eligibility.

A scorecard is Pareto-comparison eligible only when task utility, privacy, revocation, causal, resource, role (when relevant) and policy evidence are complete, and no unauthorized disclosure, forbidden inference or unreconciled descendant is observed. This boolean is a gate, not a ranking or scalar score.

## At-rest integrity and erasure-resurrection plane

MCSF adds a separate at-rest integrity qualifier aligned with the active [IETF agent memory integrity Internet-Draft](https://datatracker.ietf.org/doc/html/draft-khandelwal-bpm-agent-memory-integrity). The document is work in progress, not an RFC. Its new T10 case is especially important: the SUT legitimately erases a record, then the evaluator restores a complete store snapshot taken before that erasure. This tests whether erasure survives rollback, not merely whether the current live store hides a deleted row.

The ten cases are kept separate:

| Case | Storage-level attack |
|---|---|
| T1 | Modify a stored record's content |
| T2 | Truncate the tail of the store |
| T3 | Delete a record from the middle |
| T4 | Reorder stored records |
| T5 | Insert a forged record |
| T6 | Replay a record from another context |
| T7 | Replay an older genuine record from the same context |
| T8 | Tamper with owner, role, source, or timestamp metadata |
| T9 | Restore an older whole-store snapshot after a genuine new write |
| T10 | Restore a pre-erasure snapshot after the SUT's own erasure operation |

The evaluator must establish three controls. C1 is a stop/restart with no edit: the SUT should serve seeded memory unchanged and raise no integrity alert. C2 is a genuine write through the SUT after restart: the new record must be served. C3 is per-case edit-landed evidence from an independent raw-store comparison: the intended storage mutation must have happened, and only the intended change must be present. A failed C3 excludes that case from the denominator; it is neither a pass nor a SUT security failure. If C1 or C2 fails or is missing, the whole metric is not evaluable.

For applicable cases, read-time rejection and read-time reporting receive full credit; audit-only reporting after the read receives half credit; accepted manipulated memory receives zero and fails qualification. The score is reported only across applicable cases whose edit landed and whose outcome is known. N/A requires both a capability/evidence digest and a human-readable reason. T9 cannot be N/A when whole-store snapshot restoration is supported. T10 cannot be N/A when both snapshot restoration and the SUT's own erasure operation are supported. T9 and T10 receive their own profiles rather than disappearing into a global average.

The run commitment binds SUT identity/version/revision, store backend/version, runtime version and library digest, storage configuration, runner and test-plan digests, measurement date, detection and observation boundaries, stopped-SUT edit procedure, restore/erase capabilities, C1/C2 outcomes, all ten case identities, C3 states, outcomes and evidence digests. Every storage-layer edit must be performed while the SUT is stopped. A read-time result is observed at the independent harness or client-SDK boundary; an audit-only result is observed as a DUT-reported audit event. These pairings are encoded separately, so read-time and post-read audit detection cannot be merged into one ambiguous result. The JSON schema and synthetic fixture are under `schema/` and `fixtures/`. The fixture is explicitly marked `synthetic_fixture`; the Rust qualifier refuses to treat it as empirical SUT evidence. Passing the contract tests proves only that the evaluator contract behaves as specified. It does not prove that any production database, vector store, backup system or agent memory product has passed T1–T10.

This plane complements, but does not replace, MCSF's existing runtime privacy, role-visibility, revocation and causal-influence probes. A system can correctly deny a cross-context query while its stored files remain replayable; conversely, a tamper-evident database can faithfully serve a memory to the wrong principal. Those failure modes need independent measurements.

## Executable sparse baseline

The first matched retrieval baseline is implemented in `src/mcsf_baselines.rs`:
- `NoMemoryAdapter` provides the zero-retrieval control.
- `Bm25Adapter` implements Okapi BM25 with explicit (k_1) and (b), Unicode-aware tokenization, deterministic fixed-point score output, stable memory IDs/content digests and deterministic ID tie-breaking.
- Both implement one `McsfRetrievalAdapter` interface. Dense-vector and HDC/CfC conditions must implement that interface around their **real** embedding/HDC/CfC engines; the interface does not simulate either system.

The BM25 tests cover relevance ordering, non-matches, Top K, deterministic ties, duplicate IDs, invalid inputs and non-ASCII text. This is a retrieval baseline component, not yet a full benchmark runner or empirical MCSF result.

## Initial pilot and next scale-up

The checked-in fixture has two synthetic Textual tasks and three architectures (NoMemory, BM25 and HDC+CfC), with made-up scores and evidence digests. It exists to exercise the schema and Rust contract only.

The first actual pilot should freeze a small public or synthetic task family, then scale to the same AML Textual tasks where public raw tasks are available and permitted. Preserve the official AML evaluator and report its score separately. For native HDC/CfC comparisons, match answer model, token/context budget, hardware, Top K, concurrency, seed and write/search budget. Keep any task-family or query-policy extension separately versioned and sealed.

## Scale-sensitive external memory references

The public Mem0 benchmark repository reports managed-platform results of 94.4% on LongMemEval (500 questions), 92.5% on LoCoMo (1,540 questions), and BEAM results of 64.1 at 1M tokens versus 48.6 at 10M tokens. It reports average query token use around 6.8–7.0K and p50 latency around 0.88–1.09 seconds for LoCoMo/LongMemEval. These are vendor-reported managed-platform measurements, not an independent apples-to-apples Mycelix comparison; the repository explicitly cautions that the managed platform includes proprietary optimizations not present in the OSS SDK.

The algorithm description is an instructive design trade-off: its recent pipeline is add-only (no Update/Delete in extraction), uses entity linking and fused semantic/BM25/entity retrieval, and reports high recall. MCSF should reproduce the same LongMemEval/LoCoMo/BEAM task versions where licenses permit, but also probe whether add-only accumulation increases stale-memory, supersession, deletion and revocation failures over time. Scale suites must match source-corpus tokens and record inventories and report storage/indexing/search cost separately.

## Current external reference points

AML Cycle 2 (published 19 November 2026) reports separate track/division scores: Textual open-source Resurface 61.22; Coding open-source InsightMem 63.33% Overall Resolved; Multimodal open-source Khipu 43.26. These are not cross-track comparable. The Textual track also publishes capability scores including memory governance 76.55 and epistemic safety/privacy 68.59 for Resurface, showing why a total alone is insufficient.

Prospective targets are +2 absolute points above the strongest reproduced open-source system on the same official track, with a 95% paired interval and no task-family regression hidden by the mean; governed utility within 2 percentage points of matched unconstrained memory; 1,000 negative controls with zero observed unauthorized disclosures (with intervals, not a universal zero-risk claim); complete seeded revocation closure; and positive cost-adjusted marginal gain for each added agent.

## Claim ceiling

MCSF is a research evaluator, not a privacy certification, differential-privacy guarantee, formal noninterference proof, production memory platform, or official AML submission. The fixture is synthetic. A finite successful panel falsifies only its specified controls; out-of-view channels and real deployment effects remain untested.

## Composed owners

- #6759 MCSF benchmark programme.
- #6755 / #6758 AML Add/Search and official cycle policy.
- #6730 / #5462 / #6753 longitudinal observer-trace privacy.
- #6734 / #6719 transitive lineage and revocation closure.
- #6735 / #6742 audience/applicability/disclosure.
- #6700 / #2952 actual model-input exposure and causal influence.
- #6686 / #6746 / #6749 coordination value and TeamBench.
