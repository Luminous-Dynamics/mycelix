# Craft Temporal Evidence Graph Qualification Protocol V1

**Status:** research specification; no implementation or qualification result is claimed by this document.  
**Tracking:** [#6823](https://github.com/Luminous-Dynamics/mycelix/issues/6823)  
**Parent experiment:** [PR #6805](https://github.com/Luminous-Dynamics/mycelix/pull/6805), stacked on [PR #6796](https://github.com/Luminous-Dynamics/mycelix/pull/6796)

## 1. Purpose

Move Craft from a single-log temporal relationship projection toward a **decision-qualified temporal evidence graph** that can reconstruct what was asserted, what the platform had actually admitted, what was causally knowable, which authority and policy versions applied, what evidence supported or contradicted the claim, and why a particular operational decision was or was not justified.

The goal is not to replace OCEL, PM4Py, a temporal database, a graph database, Holochain, or the six Craft owner cores. It is to define the assurance layer that joins their outputs without confusing event order, source time, physical truth, authority, evidence independence, process conformance, or decision legitimacy.

## 2. Why the current two-frontier model is necessary but insufficient

The V1 prototype in #6796/#6805 has a useful contract:

- a domain valid-time cutoff;
- an accepted-sequence knowledge cutoff;
- append-only corrections and revocations;
- deterministic, digest-bound snapshots;
- historical queries isolated from records outside the queried knowledge frontier.

That is stronger than simply sorting an event array by timestamp. It is not yet full bitemporal storage or distributed causal replay. In particular:

1. A host-assigned sequence is an ordering coordinate, not a trusted wall-clock transaction timestamp.
2. A source-recorded timestamp is source metadata, not proof of when the platform accepted the record.
3. A single sequence assumes one append order. Real municipal water operations combine independently operated event sources.
4. Different sources may have skewed or low-resolution clocks. Equal or overlapping timestamps do not prove causality.
5. A digest commits to bytes; it does not authenticate their origin, establish authority, prove measurement independence, or establish that an operational outcome occurred.

The next model must represent a **partial order of knowledge** and make the trust boundary explicit.

## 3. Typed time and order model

Do not add another overloaded `timestamp` field. Each field answers a different question.

| Dimension | Meaning | Trusted by itself? |
|---|---|---|
| Domain valid interval | When the assertion is claimed to hold in the modeled world; use half-open intervals `[from, to)` | No; it is a domain assertion |
| Observed-at interval | When a sensor, person, laboratory or system observed the underlying phenomenon, including clock uncertainty | No; source and clock provenance must be evaluated |
| Source-recorded-at | When the source says it recorded or issued the assertion | No |
| Host admission coordinate | A platform-assigned position in a named append log and epoch, bound to the admitted bytes | Only after its admission receipt is verified |
| Causal parents | Exact prior records consumed or depended on by this record | Only after parent identities and inclusion are verified |
| Policy/authority validity | When a specific policy or capability version is effective | Only after issuer, scope, version and revocation checks |
| Evaluated-at | When an evaluator produced a decision receipt | Must be tied to evaluator and clock provenance if used as an attested time |

### 3.1 Admission coordinates are not timestamps

Represent an admission coordinate at minimum as:

`(log_id, epoch, offset, entry_digest, inclusion_receipt_digest)`

The platform or log owner assigns the coordinate. It must not be read from an OCEL event attribute and silently promoted to trusted platform acceptance. A source-supplied offset may be retained as source metadata, but it is a different field.

If an independently attested commit time exists, carry it as a separate value with clock identity, uncertainty, attester identity and verification result. Never infer it from an offset.

### 3.2 Federated causal frontier

A query frontier is a canonical set/vector of per-stream cuts, not a fabricated global counter:

`F = { (stream_i, epoch_i, offset_i, head_digest_i) }`

An event is included only if its stream coordinate is within the cut and all of its required causal parents are included. The cut must be downward-closed over verified causal dependencies. Missing parents, epoch mismatches, forks or gaps are explicit errors or incomplete-frontier states; they must not be silently skipped.

Two events from different streams are incomparable unless a verified parent/dependency edge or a separately qualified ordering fact relates them. A deterministic serialization order may be used for hashing, but it must not be presented as causal order.

A future implementation may support a scalar sequence as an optimized special case for a single append log. It must not generalize that special case into a distributed time model.

### 3.3 Clock uncertainty

Where a source cannot provide a trustworthy exact instant, represent an observation as an interval plus clock provenance, resolution and uncertainty. If two observation intervals overlap, do not decide which happened first solely from their nominal timestamp. Preserve ambiguity and let domain rules, independent evidence or human review resolve it.

## 4. Temporal evidence graph model

The graph is an append-only set of typed claims and immutable source artifacts. A projected edge is a view, not the source of truth.

### 4.1 Claim

A relationship claim should bind:

- stable claim ID and schema version;
- subject, predicate, object and declared endpoint cardinality;
- domain-valid interval;
- source observation/recording metadata and uncertainty;
- source artifact reference and content digest;
- source stream and source coordinate, where available;
- host admission coordinate and inclusion receipt, only after verified admission;
- asserting principal and the exact authority/capability reference;
- policy version and relevant jurisdiction/domain scope;
- verifier version and verification evidence;
- explicit status dimensions rather than one blended trust score.

### 4.2 Evidence and provenance are also first-class claims

Evidence can support, contradict, qualify, derive, supersede or revoke another claim. These relationships must themselves have identifiers, source references, validity, provenance and authority context. This avoids treating an evidence link as proof merely because the link exists.

A digest verifies byte identity only. Origin authentication, signature validity, identity resolution, authority scope, freshness, key status, custody, transformation lineage and independent corroboration are separate checks with separate outcomes.

### 4.3 Admission is a sequence of verifiable predicates

The proposed admission ladder is:

1. **Observed** — bytes or a source reference have been received.
2. **Parsed** — syntax and declared schema/profile are understood.
3. **Structurally valid** — identities, types, references and local invariants pass.
4. **Content-bound** — canonical content digest and source artifact binding pass.
5. **Origin-authenticated** — signature or other source-authentication mechanism verifies.
6. **Principal-qualified** — the issuer resolves to the expected identity/key at the relevant frontier.
7. **Authority-qualified** — authority is current, scoped to the action/domain/object and valid at the relevant time.
8. **Evidence-qualified** — custody, methods, calibration, transformation lineage and independence requirements are met for the particular claim type.
9. **Admitted for a declared purpose** — the policy permits this evidence to support the named decision class.

Each step can be `Pass`, `Fail`, or `Unknown` with a reason and verifier receipt. Missing data or an unavailable verifier must remain Unknown. A record can be structurally valid but not admissible evidence for a decision. Avoid collapsing these dimensions into a global confidence/reputation scalar.

### 4.4 Common-cause and evidence independence

Two records are not independent merely because they have different IDs, issuers or devices. Maintain a dependency graph for:

- shared sensor hardware, calibration and power;
- shared sampling point, technician, laboratory, method and custody chain;
- upstream data feeds, copied reports and common contractors;
- common identity, signing key, gateway or transformation pipeline.

The independence test is policy-relative and domain-specific. It should ask whether the evidence paths share a failure mode relevant to the claim, not simply count sources. Preserve dependence as a graph so policy can change without rewriting source facts.

### 4.5 Corrections, disputes and revocations

Corrections, disputes, supersessions and revocations are new append-only records bound to exact target claim digests. A mutation must identify its author, authority, scope, effective interval, causal parents and source evidence. A correction cannot silently rewrite the original. Conflicting or concurrent mutations remain conflicts until a policy-qualified resolution is admitted.

A later mutation can change a current projection, but it cannot rewrite a historical query at a frontier that excludes that mutation.

## 5. Decision closure and replay receipt

A decision receipt must commit to more than the final list of active edges. It should bind:

- query identity and domain-valid cutoff;
- exact canonical causal frontier vector and verified frontier heads;
- complete dependency closure of claims, evidence artifacts and mutations considered;
- the accepted/rejected/unknown verification outcomes and verifier versions;
- authority/capability snapshot and revocation state;
- policy/profile digest and effective version;
- process model/evaluator version, where process conformance is used;
- conflicts, unresolved dependencies, uncertainty and abstention reason;
- outcome classification and the exact evidence used to justify it.

The same input frontier, policy digest, verifier set and evaluator version must produce the same canonical receipt. A later-arriving correction should cause a *new* evaluation against a newer frontier. It must not mutate the older receipt.

### 5.1 Separate four questions

A Craft decision should preserve these as separate results:

1. **Process conformance:** does the observed event/object trace conform to the declared process model?
2. **Evidence qualification:** are the assertions and evidence admissible for this claim and purpose?
3. **Authority qualification:** did the relevant actors and institutions have valid authority for these actions at the relevant frontier?
4. **Outcome qualification:** did the physical/service outcome actually occur, and is there sufficient independent evidence for that outcome?

A trace can be process-conformant while a permit is revoked, an assessor is unauthorized, a sample's custody is broken or pressure restoration is unverified. Conversely, an operational outcome may occur even when the recorded process is incomplete. Do not collapse these dimensions.

## 6. Proposed query contract

The following is conceptual pseudocode, not a committed API:

```text
project(
    valid_at = T,
    causal_frontier = F,
    policy_digest = P,
    authority_frontier = A,
    evidence_policy = E
) -> {
    active_claims,
    known_claims,
    pending_claims,
    expired_claims,
    superseded_claims,
    revoked_claims,
    contested_claims,
    admission_failures,
    unresolved_dependencies,
    common_cause_groups,
    decision_basis_digest
}
```

Required semantics:

- reject a non-downward-closed causal frontier;
- never infer order from unrelated source clocks;
- preserve concurrent contradictory claims as conflicts;
- apply a mutation only when its event is admitted by the frontier and its domain-effective time is reached;
- evaluate authority and policy at their declared effective times and the decision's knowledge frontier;
- do not allow a future malformed record outside the frontier to poison a historical view;
- never treat absence of a record as evidence that an event did not occur unless the relevant source's completeness/watermark contract is itself qualified.

## 7. Benchmark programme

Every benchmark result must record source SHA, fixture/corpus digest, schema/profile version, frontier semantics, policy/verifier/evaluator versions, independent oracle, toolchain, OS, CPU/RAM, resource limits, raw output and receipt digest. Correctness and throughput are separate gates.

| Benchmark family | Frozen challenge | Qualification target |
|---|---|---|
| Frontier correctness | Permutation of independent streams; missing parents; gaps; forks; duplicate/rollback offsets; epoch rollover | 100% agreement with independent oracle; invalid cuts never accepted |
| Historical replay | Append late corrections/revocations after taking an earlier snapshot | 0 changes to the earlier decision/replay receipt |
| Partial-order semantics | Concurrent conflicting updates with equal/overlapping timestamps | No invented ordering or last-write-wins selection |
| Admission security | Digest-only spoof, invalid signature, stale/revoked key, wrong-scope authority, missing identity service | 0 false admissions in frozen negatives; Unknown stays non-qualified |
| Evidence independence | Correlated sensors, copied reports, common technician/lab/custody, shared upstream feed | 0 false independence classifications in the known-answer suite |
| Mutation integrity | Retarget digest, cross-object mutation, mutation without causal target, unauthorized correction, concurrent revocation/correction | Every declared negative control rejected or explicitly contested |
| Decision closure | Policy change, credential revocation, contradiction, new evidence, evaluator-version change | Exact historical replay and explicit new evaluation; no silent re-finalization |
| OCEL interchange | OCEL 2.0 JSON/XML/SQLite; experimental OCEL 2.1 CSV and bundled CSV/Parquet; typed values, object changes, E2O/O2O qualifiers and unknown extensions | 100% normalized preservation within each declared profile; 0 silent drops/coercions |
| Scale/resource | 1K, 100K, 1M and public multi-million-event corpora; varied fan-out, mutation density and stream count | Report p50/p95/p99, peak RSS, bytes/event, temp-disk use and throughput; no latency SLO until named-hardware baseline |
| Water-service outcomes | Synthetic and, later, consented field cases for repair, pressure, sampling, custody, advisory and recurrence | Report false-safe and false-block rates, revalidation delay, restoration delay and repeat-incident rate against adjudicated ground truth |

### 7.1 Scale ladder

- **1K events:** developer feedback and semantic regression.
- **100K events:** integration/resource gate.
- **1M events:** practical large-log test.
- **7M+ events:** public multi-million-event anchor on an isolated runner.
- **10M synthetic events:** stress-only.
- **Temporal graph scale:** optionally use TGB 2.0's multi-relational datasets (up to 53 million edges) for generic graph indexing/query stress, but not as a substitute for OCEL or evidence-admission correctness.

Do not set arbitrary latency promises before a reproducible baseline is collected. Publish a matched comparison against the current whole-document Rust reference and the same hardware/runtime settings. Include temporary storage and external-sort cost for streaming implementations.

### 7.2 Required baselines and oracles

- Current Rust whole-document validator/replay remains the reference oracle until a streaming engine proves exact semantic parity.
- PM4Py remains a test-only OCEL import/export and process-mining comparator. Keep known exporter type coercions visible; do not normalize away a failed round trip.
- OC-DCR discovery/alignment is a process-conformance comparator. It is not a source-authenticity, authority or physical-outcome oracle.
- A separate, small independent implementation should validate the causal-frontier and replay semantics. It must not reuse the production projection code or fixtures' expected outputs as its algorithm.
- The 16-case water-service known-answer suite and declared mutation inventory should be expanded to include the federated-stream adversarial families before claiming decision-level qualification.

## 8. Standards and industry comparison

| System/standard | What it already does well | Correct Mycelix comparison |
|---|---|---|
| OCEL 2.0 | Object-centric events, objects, object changes, E2O/O2O relationships and qualifiers | Use it as the stable interchange baseline; add a separate temporal evidence/authority layer rather than inventing an event format |
| OCEL 2.1.0pre5 | Adds compact CSV and bundled CSV/Parquet exchange layouts | Test as an experimental high-volume interchange target; do not assume its pre-release schema authenticates claims |
| PM4Py | OCEL ingestion/export, discovery, filtering and process analytics | Match interchange and process analytics; measure and report exporter losses explicitly |
| OC-DCR (2026) | Formal object-centric declarative process models and alignment-based conformance | Compare process discovery/conformance head-to-head; do not claim superiority from unrelated evidence checks |
| XTDB | Bitemporal valid-time/system-time queries and retroactive corrections | Use as a temporal database semantic/performance comparator; its two time axes do not by themselves supply Craft's federated evidence-admission and domain-authority contract |
| W3C PROV-O | Interoperable provenance model for entities, activities and agents, including qualified influence relations | Map Mycelix provenance to existing terms where possible; specify only the water/operational extensions actually needed |
| TGB 2.0 | Reproducible temporal-graph learning benchmarks and multi-million-edge datasets | Reuse relevant scale/temporal-graph workload ideas; its link-prediction metrics are not decision-qualification metrics |
| Apache Iceberg | Snapshot lineage and reproducible point-in-time table access | Consider its snapshot/manifest discipline for data-lake benchmark artifacts, not as the authority or evidence model |

### 8.1 Strategic differentiation

Do not compete with PM4Py by trying to become the best general-purpose process discovery package. Do not compete with XTDB by building a generic temporal database before demonstrating need. The narrower, defensible thesis is:

> **Process conformance tells us whether recorded behavior matches a model. Temporal evidence qualification tells us whether the facts, provenance, authority, dependencies and physical outcomes justify a consequential decision at a precise knowledge frontier.**

A work order can conform perfectly while a permit is revoked or its assigned crew qualification is stale. A lab result can be syntactically valid while its sample custody chain is broken. Two pressure sensors can agree while sharing the same failed calibration source. These are the cases the benchmark must expose.

## 9. Implementation sequence

1. **Freeze semantics first:** publish the frontier and admission vocabulary plus a compact known-answer fixture. Do not modify the V1 two-frontier contract while doing this.
2. **Independent oracle:** implement the causal-cut validator and replay semantics independently; pin its toolchain and produce machine-readable receipts.
3. **Rust prototype:** add a separate experimental module with per-stream offsets, epoch/head digests, causal-parent closure and explicit incomparable-event conflicts. Keep source-supplied offsets separate from host admission receipts.
4. **Evidence dependency graph:** model common-cause and custody/transform dependencies; add adversarial fixtures before a policy engine.
5. **Decision closure:** bind exact frontier, policy, authority and evaluator versions into replayable decision receipts.
6. **Streaming:** only after reference semantics stabilize, add bounded-memory parsing, external sorting and streaming digest generation under #6803.
7. **Field bridge:** begin only with consented, purpose-limited, independently adjudicated water-service data. Synthetic passes do not qualify municipal outcomes.

## 10. Claim boundaries

This specification is not an implementation, test pass, performance result, security audit or field trial. A deterministic digest does not establish truth. A valid signature does not establish legal authority. A qualified sample does not prove water was restored. A process-conformant trace does not prove the process was safe or legitimate. No result should be described as SOTA-leading until a matched, reproducible comparison exists.

## 11. References

- [OCEL 2.0 specification and formats](https://www.ocel-standard.org/2.0/ocel20_specification.pdf)
- [OCEL 2.1.0pre5 specification, dated 1 October 2026](https://www.ocel-standard.org/2.1/ocel20_specification.pdf)
- [OCEL 2.1 bundled CSV/Parquet format](https://www.ocel-standard.org/specification/formats/bundled/)
- [PM4Py OCEL 2.0 import/export API](https://github.com/process-intelligence-solutions/pm4py/blob/release/docs/source/api.rst)
- [Christfort et al., Discovery and conformance checking of object-centric DCR graphs from event knowledge graphs, published 24 November 2026](https://link.springer.com/article/10.1007/s44311-026-00061-9)
- [XTDB bitemporality](https://v1-docs.xtdb.com/concepts/bitemporality/)
- [W3C PROV-O Recommendation](https://www.w3.org/TR/prov-o/)
- [TGB 2.0, NeurIPS 2024 Datasets and Benchmarks Track](https://proceedings.neurips.cc/paper_files/paper/2024/hash/fda026cf2423a01fcbcf1e1e43ee9a50-Abstract-Datasets_and_Benchmarks_Track.html)
- [Apache Iceberg specification](https://iceberg.apache.org/spec/)
