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

## Non-compensatory output vector

Each architecture scorecard separately reports:
- native task utility and paired delta against NoMemory on matched task IDs;
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

## Initial pilot and next scale-up

The checked-in fixture has two synthetic Textual tasks and three architectures (NoMemory, BM25 and HDC+CfC), with made-up scores and evidence digests. It exists to exercise the schema and Rust contract only.

The first actual pilot should freeze a small public or synthetic task family, then scale to the same AML Textual tasks where public raw tasks are available and permitted. Preserve the official AML evaluator and report its score separately. For native HDC/CfC comparisons, match answer model, token/context budget, hardware, Top K, concurrency, seed and write/search budget. Keep any task-family or query-policy extension separately versioned and sealed.

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
