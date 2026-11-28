# Craft Temporal / DJCC Composition Contract V1

**Tracking:** [#6827](https://github.com/Luminous-Dynamics/mycelix/issues/6827)  
**Parent research protocol:** [#6823](https://github.com/Luminous-Dynamics/mycelix/issues/6823)  
**Draft integration PR:** [#6825](https://github.com/Luminous-Dynamics/mycelix/pull/6825)

## Purpose

Compose existing temporal, source-trust, evidence-independence and decision-justification owners into one Craft water-service decision path without creating a duplicate temporal database, provenance graph, transparency log, policy engine or action ledger.

The integration target is a content-addressed **Decision Basis Capsule (DBC)**. The capsule references exact outputs from existing owner systems and exposes missing, conflicting or unqualified inputs. It is not a second event store.

## Existing work to reuse

| Owner/workstream | Existing contribution | Boundary that must remain visible |
|---|---|---|
| #6796 / #6805 | OCEL exchange, single-log valid-time + accepted-sequence replay, temporal relationship claims | Restricted profile; host sequence is not trusted wall-clock transaction time; parser is not full OCEL schema conformance |
| #6804 | Verifier-relative progress antichain over owner-local partial-order progress | Does not authenticate progress, source universe or real-world absence |
| #6822 | DJCC stages, exact producer-consumer digest/schema bindings, claim-ceiling non-widening, structural failure-domain graph | Does not authenticate stage producers, owner-universe completeness, legal authority or physical outcome |
| #6826 | Follow-on failure-domain lineage attestation and threat-model-relative independence | Required before structural disjointness can become an attested independence claim |
| #6819 / #6792 | Immutable authority-bound action denominator and exact plan revisions | Does not establish completed work or service restoration |
| #6790 | Water-service cross-core harness and independent synthetic service-level oracle | Synthetic scenario qualification only |
| #6762 | Correction, expiry, supersession and revocation impact propagation | Consumes admitted changes; it is not an event source or trust authority |

## Critical distinction: causal event frontier versus progress antichain

These are different partial-order objects and must have different types, schemas, receipts and claim ceilings.

**Causal event frontier**: a per-stream cut over admitted source-log coordinates. It answers which records are included in a distributed knowledge snapshot. It contains stream identity, epoch, contiguous accepted offset/prefix, committed head digest and verified causal-parent closure. Events on unrelated streams can remain incomparable even when their timestamps are equal.

**Progress antichain**: a set of pairwise incomparable minimal progress points over a declared owner universe. It answers whether any declared progress point can still precede a requested cut. #6804's implementation calls its stable result only `StableRelativeToDeclaredProgress`; it keeps progress authentication and owner-universe completeness NotEvaluated.

The antichain's product-order operations may inform the causal-frontier implementation, but an antichain is not a vector clock, not an authenticated event log and not proof that every source or owner is represented. A complete event frontier can coexist with an incomplete owner universe; a complete event frontier can also coexist with provisional owner progress.

## Proposed DBC stage graph

The Craft profile should bind the following owner outputs as DJCC producer/consumer stages:

1. **Verifier frontier** — accepted log identities, epochs, contiguous offsets, prefix/owner coverage, checkpoint/head digests, causal-parent closure and the separately typed progress-antichain result.
2. **Source trust** — exact artifact bytes/digests, issuer/key identity, signature or VDS receipt verification, registry/policy epoch and key status at the selected frontier.
3. **Semantic projection** — OCEL objects/events/attributes and temporal relationship claims evaluated at the selected valid time and causal frontier.
4. **Query** — the explicit service question, exact scope, source/claim set, required evidence and applicability context.
5. **Policy** — historical policy/profile digest and evaluation result.
6. **Authorization** — scoped decision-maker, operator, assessor and jurisdiction authority at the same historical frontier.
7. **Execution** — immutable action-plan digest/revision, action-scoped authorization, idempotency scope and execution transcript.
8. **Outcome** — independently qualified pressure, water-quality, repair, custody and service-outcome evidence; unresolved outcome must remain a mutation barrier.

The existing DJCC V0.1 stage order and required edge set remain authoritative. This document does not redefine or weaken them. Each edge must retain exact producer output digest, schema reference, named consumer input and required claim dimensions.

The six DJCC claim dimensions remain independent:

- Authentication
- Authority
- Completeness
- Effect
- Independence
- Truth

The Rust enum's current states are `EstablishedWithinProfile`, `Rejected`, `Indeterminate`, `NotEvaluated`, `UnsupportedByProfile` and `NotApplicable`. A downstream stage must never widen any claim ceiling. In particular:

- Authentication does not imply authority.
- An authenticated source list does not automatically imply owner-universe completeness.
- Structural failure-domain disjointness does not imply authenticated independence.
- Action coverage does not imply execution.
- Execution acknowledgement does not imply physical effect.
- Process conformance does not imply evidence truth or safe outcome.

## Decision Basis Capsule V1 (conceptual)

The DBC is a compact, canonical, content-addressed composition envelope. It references receipts instead of copying source payloads.

Required binding groups:

| Group | Required binding |
|---|---|
| Identity | DBC schema/profile digest, decision ID, query ID, water-service episode and exact scope |
| Time | Domain valid-time query, source observation-time uncertainty, accepted frontier vector digest; never one overloaded timestamp |
| Causal frontier | Per-stream log ID, epoch, accepted offset, committed head and parent-closure receipt |
| Progress | Progress-antichain digest, declared owner universe and independent completeness disposition |
| Interchange | OCEL profile/version, source corpus digest, normalizer version and exact replay receipt |
| Evidence | Source trust receipts, claim/evidence dependency closure, correction-impact receipt, sample custody and method/calibration references |
| Independence | Failure-domain graph digest, threat-model/profile digest, lineage attestation and accepted verifier frontier |
| Governance | Policy digest, domain-standard revision, jurisdiction, authority identity/scope/validity and revocation state |
| Work | Immutable action-plan digest/revision, per-action risk/role/prerequisite/qualification profile and coverage receipt |
| Execution/outcome | Execution transcript digest, idempotency scope, outcome evidence and exact reconciliation receipt where required |
| Verdict | Six-axis claim ceiling, disposition, conflicts, unknowns, abstention reason, evaluator/build digest and canonical DBC digest |

No field may contain a bare `trusted=true` flag. Every positive claim must name the profile, evidence and verifier basis that establishes it. Missing references remain NotEvaluated or Indeterminate.

## Required composition fixtures

The machine-readable seed manifest is `fixtures/craft-temporal-djcc-integration-v0.1.json`. It is a specification-only known-answer corpus; there is no executable DBC evaluator or qualification claim yet.

The first 13 seeds cover:

- concurrent cross-stream assertions with equal timestamps;
- missing causal-parent/prefix coverage;
- conflicting stream heads at one epoch/offset;
- complete event frontier with provisional progress;
- incomplete owner universe despite a locally complete frontier;
- common upstream/calibration failure domain;
- different vendors/keys sharing one upstream data source;
- opaque or withheld failure-domain lineage;
- valid signature with expired or wrong-scope authority;
- retroactive policy change and historical replay;
- action-plan denominator amendment;
- unknown external outcome and same-scope mutation barrier;
- PM4Py integer-declaration coercion.

Every seed has an expected disposition and claim ceiling. Positive controls must be added alongside each negative family before a rate is reported; otherwise the benchmark could reward rejecting everything.

## Benchmark design

Report the following independently, never as a single weighted accuracy score:

- false-safe decision rate;
- false hold / unnecessary-block rate;
- false absence and false completeness;
- false authentication and false authority;
- false independence and false outcome;
- historical receipt mutation;
- correction-impact precision and recall;
- abstention quality and recovery/requalification latency;
- reviewer burden and dispute-resolution time;
- p50/p95/p99 latency, peak RSS, input/output bytes, temporary storage and proof/verification cost.

### Scale and baselines

1. Reuse the BPIC-derived OCEL scale anchors: approximately 0.69M to 7.36M events and 0.1M to 0.75M objects, but report workload topology and relation counts alongside raw event counts.
2. Use TGB 2.0 only for generic temporal-graph indexing/query stress (not process conformance or evidence qualification).
3. Compare process discovery and alignment with PM4Py and OC-DCR on the same OCEL corpora, preserving their official data splits and preprocessing requirements.
4. Compare time/query semantics with XTDB and other bitemporal systems only on matched query profiles.
5. Compare trust/composition against DJCC, SCITT/VDS/RFC 9162 owner implementations using the same receipt bytes and exact frontier assumptions.
6. Establish the existing whole-document Rust implementation as the semantic reference before evaluating #6803 streaming/external-sort implementations.

The initial scale ladder remains 1K, 100K, 1M, 7M+ public logs and a 10M synthetic stress workload on isolated, named hardware. Do not publish an SLO until repeated, controlled baselines exist.

## Integration sequence

1. Keep #6796/#6805 and #6822 owner semantics independent; record exact current source heads and qualification receipts.
2. Extend the independent known-answer corpus and mutation manifest before creating a composite evaluator.
3. Define a JSON Schema and canonical DBC byte contract; add Go/Rust known-answer vectors generated independently.
4. Add an adapter-only prototype that composes existing owner receipts without copying the evaluators.
5. Add the causal event-frontier owner and separately verify its downward-closed cuts; use #6804 only as partial-order algebra reference.
6. Require #6826-style authenticated, complete failure-domain lineage before any positive independence claim.
7. Integrate exact action-denominator revisions and the unresolved-outcome barrier.
8. Only after reference parity, add streaming replay and evaluate the full water-service corpus.
9. Keep production zome integration behind separate authority, privacy, professional-safety and field-validation gates.

## Claim ceiling

A passing synthetic DBC test demonstrates only deterministic composition under the declared profile, fixture and owner versions. It does not prove source authenticity, complete municipal source coverage, legal authority, professional qualification, water-quality compliance, physical restoration or field effectiveness. Those claims require independently qualified trust roots, source inventories, professional adjudication and consented field evidence.

## References

- OCEL 2.0/2.1 specifications and public event-log catalogue: https://www.ocel-standard.org/
- OC-DCR discovery and alignment paper (published 24 November 2026): https://link.springer.com/article/10.1007/s44311-026-00061-9
- TGB 2.0: https://proceedings.neurips.cc/paper_files/paper/2024/hash/fda026cf2423a01fcbcf1e1e43ee9a50-Abstract-Datasets_and_Benchmarks_Track.html
- XTDB bitemporal model: https://docs.xtdb.com/about/time-in-xtdb.html
- W3C PROV-O: https://www.w3.org/TR/prov-o/
