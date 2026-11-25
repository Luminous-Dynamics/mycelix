# Relational Evidence Refinement v0.1

**Status:** deterministic reference transformation and research protocol. No GroupAssistBench run, SOTA result, source-truth certification or production deployment is claimed.

## Research motivation

GroupAssistBench distinguishes memory retrieval from memory use. Its 80,299-message, 50-group dataset contains L1 recall and L2 open-ended assistance tasks, including single, distributed and conflicting memory dependencies (M1/M2/M3). The paper reports that full-context GPT-5.6-terra L2 assistance falls from 0.505 on M1 to 0.303 on M3. It identifies two complementary bottlenecks:
- **Relational Evidence Refinement (RER):** connect retrieved memories by subject, event, decision, chronology, update and conflict; follow relations to add missing supporting evidence from the eligible candidate pool.
- **Assistance Constraint Binding (ACB):** bind each available fact to subject, scope, status, applicability and source, then require generation to apply, exclude, condition or preserve conflict explicitly.

The paper reports positive L2 gains for both interventions across xMemory, EverMemOS and LightMem. For example, xMemory's average L2 gain was +2.19 percentage points with RER, +3.94 with ACB and +4.64 combined; EverMemOS reported +1.77, +3.16 and +4.43 respectively; LightMem reported +6.28, +2.38 and +7.03. These are results in the paper's evaluation setup, not Mycelix results. Source: [GroupAssistBench, arXiv:2611.04867v2](https://arxiv.org/html/2611.04867v2), §5 and Appendix F.2.

The intervention analysis also separates their mechanisms: RER helps most where supporting evidence is missing from the retrieval set; ACB helps where evidence is already present but not applied. Combining them is useful because the two mechanisms are complementary, not interchangeable.

## Existing Mycelix ownership to reuse

Mycelix already has:
- **#6619 evidence ledger:** source holder vs source origin, request creation/delivery, rights-aware disclosure, evidence verification, recipient exposure, claim revisions and decisions.
- **#6619 response constraint binding:** requires visible evidence to be accounted for, binds source digest/subject/scope/status/applicability, and rejects incompatible handling.
- **#6619 independent response audit:** checks rendered material claims against independently annotated evidence/source origins.
- **#6089 distributed information acquisition:** owns source-holder discovery, inquiry, delivery, rights and provenance.
- **#6677 source-origin counterfactuals:** owns matched origin-removal replay.

This increment adds only the missing **bounded relational selection transform** between retrieval and response-constraint binding. It does not create another store, ledger, source registry, authorization engine, semantic truth oracle, or language model.

## Pipeline placement

`eligible candidate pool → relational refinement → response constraint binding → generation → independent rendered-response audit`

RER answers: "Which authorized, verified evidence is related to the retrieved seeds and should be considered together?"

ACB answers: "Given the evidence that is now visible, how must each item be applied, excluded, conditioned or preserved as a conflict?"

Neither stage proves factual truth merely because a relation or source receipt exists.

## Reference implementation

`src/relational_evidence.rs` provides `refine_relational_evidence`.

Inputs:
- a request-bound scope with recipient, purpose and frozen history cutoff; each candidate record is bound to that same request context;
- a set of candidate evidence records with distinct evidence ID, source origin, source holder and source message identities;
- source, payload and disclosure receipt digests;
- verification, evidence status and applicability;
- explicit recipient/purpose permissions;
- verified relation edges and their relation receipts;
- independent completeness receipts for both the candidate pool and relation index;
- a seed evidence set from the original retrieval.

Relation types include same subject, same event, same decision, support, contradiction, supersession, update and related context. Verified edges are traversed bidirectionally for selection; their original direction and type are preserved in the returned edge records.

### Rights and history rules

- Only `Shareable` or `Scoped` records authorized for the exact recipient **and** purpose may be selected.
- Rights-withheld, unknown-permission, rejected, unverified, unauthorized and future records are never added as related evidence.
- A rights-withheld or unauthorized item that is already present in the retrieval seed set fails qualification; RER does not launder an upstream exposure.
- Unknown permission or verification on a seed remains `INDETERMINATE`.
- Both source timestamp and recorded timestamp must be no later than the frozen history cutoff.
- Superseded, disputed, expired and unknown evidence status is preserved for ACB. RER does not silently delete conflicting or historical facts.
- Source origin is deduplicated separately from source holder. Multiple people repeating one source do not create independent corroboration.

### Completeness and bounded traversal

A successful selection requires independent receipts attesting that the candidate pool and relation index are complete for the request, recipient, purpose and cutoff. Merely exhausting an incomplete pool is not evidence that no related fact exists.

Traversal is bounded to 1–8 relation hops and 1–256 selected items. If a bound truncates a connected eligible closure, the result is `INDETERMINATE` and **no partial evidence bundle is emitted**. The same applies when either completeness receipt is missing or says the pool/index is incomplete.

The implementation returns stable sorted evidence IDs, added IDs, unique source origins, distinct holders, selected verified relation edges and aggregate counts. It does not return restricted candidate identifiers. It does not authenticate receipt bytes; the existing ledger/independent receipt owner must validate those upstream.

## Composition with constraint binding

For every selected record, the integration adapter maps:
- evidence ID and payload digest;
- subject and scope;
- status and applicability;
- exact authorized responder;
- whether the item is required for the response.

The existing `qualify_response_grounding` function then requires each required visible item to be handled consistently:
- current + applicable → applied;
- conditional → applied with an explicit condition;
- not applicable or superseded/expired → explicitly excluded with rationale;
- disputed → preserved as a conflict;
- unknown or unverified → unresolved.

RER expands and organizes the evidence bundle; ACB governs the response's use of the resulting bundle. The caller passes the returned constraints to the existing `qualify_response_grounding`; a successful RER pass does not imply an ACB pass.

The reference module contains 32 declared Rust tests, including direct adapter tests for non-PASS rejection, request binding, subject/scope mapping and missing selected records.

## Targeted GroupAssistBench programme

First reproduce the official benchmark and baselines:
- preserve official task definitions, history cutoffs, M1/M2/M3 labels, model, evaluator/rubric and resource budgets;
- run the official memory and full-context baselines;
- reproduce L1 recall and L2 assistance separately;
- report the official task-level uncertainty and group-clustered uncertainty.

Then run an ablation matrix under a pinned same-backbone, equal-resource envelope:

| Condition | Retrieval | Relational refinement | Constraint binding |
|---|---|---|---|
| A | Baseline | Off | Off |
| B | Baseline | On | Off |
| C | Baseline | Off | On |
| D | Baseline | On | On |

Also report a native-resource cost/quality frontier separately. Do not claim a matched win where tokens, compute, task split, evaluator or model differ.

Primary outcomes:
- L2 mean and M1/M2/M3 scores;
- evidence-anchor coverage and evidence acquisition completeness;
- unique source-origin coverage and repeated-origin inflation;
- conflicting-evidence preservation and stale-state rejection;
- material-claim source attribution and constraint-handling accuracy;
- unauthorized recipient exposure and silent evidence omission;
- cost/query, total deployment cost including construction, tokens, p95 latency and human review time.

### Prospective targets (not achieved results)

- zero unauthorized disclosures and zero repeated-origin corroboration inflation in the qualification corpus;
- 100% of visible required evidence is explicitly handled by ACB qualification fixtures;
- ≥95% material claim/source-origin attribution coverage;
- positive, paired and group-clustered L2 improvement over the strongest reproduced same-backbone baseline on M2/M3;
- RER's benefit is largest on task families with independently measured missing-support-evidence bottlenecks; ACB's benefit is largest where evidence is present but under-applied;
- the combined condition should improve the full quality/privacy/cost Pareto frontier, not merely L2 mean.

Do not set a numeric SOTA margin until the exact official release, baseline, split and scorer have been reproduced. Report confidence intervals and effect heterogeneity rather than comparing headline means from different setups.

## Limitations and claim ceiling

The reference transform trusts the supplied candidate-pool and relation-index completeness receipts and the declared verification state. It does not authenticate those receipts, prove the relation classifier is correct, discover all latent facts, judge the truth of source content, or parse a natural-language response. Candidate-pool completeness and relation accuracy remain external assurance obligations.

A `Pass` means that the returned bundle is a deterministic, bounded selection of the supplied verified and authorized graph under the declared cutoff. It does not mean the bundle is complete in the world, that all selected facts are true, or that the final answer is correct.

## Integration

- #6089 owns acquisition, source permissions, delivery and provenance.
- #6619 owns the append-only evidence ledger, response constraint binding and independent response audit.
- #6676 owns dynamic human/team capability outcomes.
- #6677 owns source-origin counterfactual replay.
- #6678 owns benchmark/evaluator validity.
- #6656 owns GroupAssistBench comparator parity.
- #6685 owns non-compensatory quality/safety/cost Pareto comparison.
- Symthaea SAT #13574 owns the wider system-level sovereignty and adaptive-control tournament.

This module is an adapter-quality increment in that architecture, not a replacement for the owning systems.
