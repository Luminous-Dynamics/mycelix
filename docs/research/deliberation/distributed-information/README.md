# DELIB-DIST-INFO-001 v0.1 — latent information and evidence pooling

**Owner:** Mycelix #6089  
**Status:** deterministic synthetic contract qualification only

## Why this benchmark exists

A group can speak extensively and still fail to discover who holds the one decision-critical fact. It can also hear the same source repeated by several agents and mistake repetition for independent corroboration. Deliberation quality therefore needs a separate measurement for information discovery, rights-safe elicitation, provenance, verification, and justified stopping.

This v0.1 package contains 18 fictional traces across six information structures. It is not an agent simulation, participant study, or evidence that any live system already performs these behaviours.

## Current external comparators

### HiddenBench — ICML 2026 / PMLR 306

Li, Naito and Shirado, *Systematic Failures in Collective Reasoning under Distributed Information in Multi-Agent LLMs*:
https://proceedings.mlr.press/v306/li26ej.html

The paper reports a 65-task HiddenBench and 15 evaluated frontier LLM configurations. In the reported setting, multi-agent accuracy was 30.1% with distributed information versus 80.7% for a single agent given the full profile. It identifies latent information asymmetry and premature convergence as key failure modes; it also reports gains from a structured communication protocol. These numbers are paper-specific comparators, not results reproduced by this package.

Official implementation: https://github.com/Yassellee/HiddenBench_ICML  
Separate MIT-licensed evaluation toolkit: https://github.com/metavert-io/hiddenbench

### CrossProfile — arXiv preprint (27 October 2026)

Imel et al., *Rare, Relevant, and Still Wrong: How Memory Fails LLM Group Decisions*:
https://arxiv.org/abs/2610.37624

The preprint describes 60 templates / 480 tasks that cross ownership of supporting, decoy and irrelevant facts while keeping fact text fixed. It reports predictable failure modes for rare-first and relevance-first memory selection, plus sensitivity to source-attribution/social-proof tags. Treat it as a preprint diagnostic, not settled SOTA. No CrossProfile text or participant payload is imported here.

## Paradigm shift

Measure the full path:

`assigned evidence → visible evidence → information gap → request → delivery → response → evidence receipt → verification → synthesis → decision readiness → stopping`

Do not collapse any of these into a binary "shared" state. A request created is not a delivered request. A statement received is not a verified fact. A repeated claim is not a new source. A rights-based refusal is not a reasoning failure.

## Fixture families

1. Complementary facts: multiple agents hold non-redundant decision-critical facts.
2. Minority decisive fact: the shared majority is incomplete without a uniquely held fact.
3. High redundancy: repeated evidence and source-origin deduplication.
4. Silent / late holder: targeted requests, non-delivery and no response.
5. Stale / conflicting evidence: superseded versions and current contradiction.
6. Rights-constrained evidence: withheld or unknown permission, correct deferral, and an unauthorized-disclosure negative control.

## What the Rust qualifier checks

- Candidate initial views contain only shared facts and that agent's own private facts.
- The oracle and complete holder map remain outside each candidate view.
- Requests identify the actual holder; creation, delivery, response and receipt are separate.
- A response is not allowed to precede delivery.
- A receipt binds the fact, holder, evidence version, origin and rights status.
- Duplicate records with the same origin count once for independent-source totals.
- Superseded, disputed, unverified, unknown-rights or withheld evidence is not silently promoted to an admitted decision premise.
- A completeness claim fails if a shareable critical fact remains missing.
- Conflicting current evidence must remain unresolved and defer the decision in these bounded fixtures.
- Rights violations produce INVALID; rights refusal / unknown rights / non-delivery with unresolved critical information produce INDETERMINATE, not PASS.

The qualifier does not execute agents or measure actual elicitation recall, query efficiency, latency or token costs. Those remain deferred experiment metrics.

## Local run

```bash
cargo test --manifest-path docs/research/deliberation/distributed-information/qualifier/Cargo.toml

cargo run --manifest-path docs/research/deliberation/distributed-information/qualifier/Cargo.toml -- \
  docs/research/deliberation/distributed-information/DIST_INFO_PROTOCOL_V0_1.json \
  docs/research/deliberation/distributed-information/DIST_INFO_PROTOCOL_V0_1.schema.json \
  docs/research/deliberation/distributed-information/DIST_INFO_FIXTURES_V0_1.json \
  docs/research/deliberation/distributed-information/DIST_INFO_FIXTURES_V0_1.schema.json
```

The Rust validator implements the bounded contracts; it is not a complete JSON Schema 2020-12 engine.

## Next experimental stage

1. Read-only HiddenBench adapter after exact data/license/custody review.
2. Reproduce HiddenBench under matched model, task, agent, round, communication and compute budgets.
3. Add a crossed ownership matrix inspired by CrossProfile: fact text fixed; ownership and source attribution varied independently.
4. Compare unstructured discussion, round-robin, broadcast-all, structured elicitation, targeted requests and rights-aware stopping.
5. Report accuracy and proper scores separately from critical-fact coverage, independent-source precision, redundancy, unasked-fact rate, request delivery, verification, query cost and worst-condition performance.
6. Only then propose a human study under separate ethics, consent, accessibility, compensation, withdrawal and retention controls.

## Claim ceiling

Passing synthetic qualification validates only bounded evidence-custody, event-state and decision-readiness invariants. It does not establish model performance, human collective cognition, democratic legitimacy, legal compliance, production readiness or authorization to compel disclosure.


## CrossProfile extension design (not yet run)

The next benchmark stage is specified separately in [CROSSPROFILE_EXTENSION_DESIGN_V0_1.md](CROSSPROFILE_EXTENSION_DESIGN_V0_1.md) and its machine-readable [ownership matrix](CROSSPROFILE_OWNERSHIP_MATRIX_V0_1.json). It independently crosses support, decoy and irrelevant fact ownership while freezing fact text and the answer oracle, then adds a separate attribution-visibility control. This design-only artifact contains no reused CrossProfile task text and does not claim an executed 480-task reproduction.

The extension is intended to detect whether a method preserves *balanced, source-grounded evidence*, not whether it blindly prefers rare facts, frequent facts, or on-topic facts. Primary comparisons will include fixed-token and fixed-item-count controls; average accuracy alone is insufficient without worst-ownership-cell and evidence-retention results.


## External comparator ledger

The structured comparator ledger and schema are maintained in:
- `EXTERNAL_COMPARATOR_LEDGER_V0_1.json`
- `EXTERNAL_COMPARATOR_LEDGER_V0_1.schema.json`

The ledger separates benchmark role (information elicitation, memory compression, forecasting, communication control, nudge sensitivity, group-memory use) from the permission and custody state of external task/result artifacts. Paper results are comparison targets, not reproduced results. Public code licenses are not treated as blanket data-subject consent or source-material provenance. No external participant records or raw conversation corpus are imported by this PR.


### GroupMemBench metadata intake

The metadata-only intake record is pinned in `GROUPMEMBENCH_METADATA_INTAKE_V0_1.json` at upstream commit `461f00c3e8a1a4a3a2ed5814b729014502c4702f`. It records 30 upstream file identities (Git blob SHAs and byte sizes): four synthetic conversation files totaling 158,659,734 bytes and 24 question files covering 715 questions. The official repository declares MIT for code and CC BY 4.0 for the synthetic conversation/question data. No payload was downloaded or committed.

This is a read-only adapter candidate, subject to preserving attribution and the pinned revision. It is not an adapter execution or a reproduced benchmark result.

GroupAssistBench is different: its surfaced repository summary describes real-world group conversations, CC BY-NC 4.0 dataset terms and academic-use-only code restrictions. Keep raw content out of commercial/production Luminous products on that release; use published aggregate findings only unless a separate permission basis is established.
