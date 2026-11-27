# Water Service Repair cross-core benchmark protocol V1

Status: proposed qualification protocol with a six-test executable slice. No field pilot, real service outcome, or comparative performance result is claimed.

## 1. Research question

Can a frozen water-service episode be translated through the six authoritative owner contracts without treating a provider status, missing evidence, mirrored sources, or stale observations as proof of physical restoration or permission to close the incident?

The composition target is:

`episode → Need Intake → Project Formation → Claim Support → Impact Qualification → Evidence Lineage → Decision Closure → independent oracle`

Every owner result remains visible, including abstention and safety-gate dispositions. The cross-core receipt binds each owner result to its source revision, packet digest and evaluator revision. It is an audit artifact, not an authority token.

## 2. Baselines and comparators

Compare semantic responsibilities separately.

| Comparator | What it does well | What must be matched before comparison |
|---|---|---|
| Conventional CMMS / work-order workflow | Assignment, work packages, job status, contractor completion, SLA and maintenance history | Same work order, authorization denominator, evidence freshness and explicit difference between reported completion and observed restoration |
| ITSM / incident-management workflow | Incident lifecycle, routing, escalation, change records and response-time reporting | Same incident events, ownership, escalation rules and closure definition |
| Event-sourced / append-only journal | Replayable event history and state reconstruction | Same correction, revocation, identity, idempotency and historical replay semantics |
| OCEL 2.1 / process-mining workflow | Object-centric event extraction, process discovery and conformance/deviation analysis | Same normalized events, object identities, source timestamps and provenance |
| Human-reviewed source packet | Independent domain judgement against source material | Blinded, pre-adjudicated outcomes, reviewer qualifications and recorded reasons |
| Mycelix six-owner composition | Typed responsibility boundaries, evidence support, impact sufficiency, lineage invalidation and conclusion-relative closure | Exact same source packets, revisions, authority scopes and expected dispositions |

A conventional workflow may be operationally excellent without claiming cryptographic or semantic completeness. Do not penalize it for failing obligations it was not designed to claim; report the obligation gap instead. Conversely, do not let Mycelix win by treating an owner abstention as success or by lowering the comparator's burden.

Useful external workload references:
- OCEL's deterministic synthetic relational databases provide six source-system scenarios with canonical OCEL references, including duplicates, corrections, retries, historical rows, projections and provenance. Preserve the exact package checksum and seed; the reference OCEL is an external target, not a proof of Mycelix's own extraction correctness.
- OCEL process-log collections include small, medium and large logs; use a small workload for iteration and a larger workload for scale only after semantics are frozen.
- The AWWA Utility Benchmarking Program publishes water/wastewater performance indicators stratified by utility type, size and region. Use these indicators for outcome reporting, not as a substitute for adjudicating the correctness of evidence decisions.

## 3. Current industry and interoperability baselines

The cross-core benchmark must compare against operational platforms and event-data standards on the capabilities they actually provide. It must not assume conventional work-order products lack workflows, AI, scheduling, evidence fields, audit history or enterprise integrations.

| Baseline | Publicly documented capabilities | Matched comparison question |
|---|---|---|
| IBM Maximo Application Suite | Work-order management, workflow scheduling/approvals, mobile work execution, asset/inventory context, AI-assisted missing-data and duplicate-work-order analysis | On identical work orders and evidence packets, do Maximo and Craft preserve the same authority, source-version, correction, scope and restoration distinctions? |
| SAP Field Service and Asset Management | AI-assisted planning/scheduling/dispatch, skills/location/priority matching, mobile and offline execution, guided work, forms and operations feedback | Compare execution throughput, offline reconciliation, work-order status and the ability to preserve the independent evidence/closure distinctions without disabling native scheduling capabilities |
| ServiceNow Field Service Management | AI-supported technician assignment, work scheduling/dispatch, mobile execution, connected asset/work history, workflow automation and enterprise integration | Compare case-to-work-order handoff, service lifecycle visibility, repeat visits, closeout evidence and context-preserving downstream impact decisions |
| OCEL 2.0 process/event representation | Object-centric events and relationships; event-to-object and object-to-object relationships; changing object attributes and relationship qualifiers | Can the same service episode be exported/imported without losing its many-object relations, event times, correction history or authority/evidence references? |
| Human-reviewed operational workflow | Existing domain/authority judgement and local context | Compare adjudicated decisions, reviewer minutes, reasons, false holds and false-safe outcomes under blinded identical inputs |

These products are real incumbents, not straw-man baselines. IBM, SAP and ServiceNow each publicly describe meaningful asset/work-order, mobile, scheduling and workflow capabilities. The experiment must use configured systems, record product/version/configuration and match access, input quality, staffing and time budgets. A capability that the configured product does not expose through the test integration must be labelled untested, not assumed absent.

Official product references:
- IBM Maximo: https://www.ibm.com/products/maximo/asset-management
- SAP Field Service and Asset Management: https://www.sap.com/products/scm/field-service-and-asset-management.html
- ServiceNow FSM: https://www.servicenow.com/products/field-service-management.html

### OCEL interoperability: reuse the event-log standard

The six-core composition is not a replacement for object-centric event logging or process mining. The benchmark should map the lifecycle into **OCEL 2.0** as the stable compatibility baseline: events can relate to multiple objects, objects can have changing attributes, and object-to-object links can exist independently of a shared event. This fits the one-to-many relationships among service episodes, complaints, assets, zones, work orders, crews, permits, samples and water-quality results.

A newer **OCEL 2.1.0pre5** specification dated 1 October 2026 is available as a pre-release. It describes compact CSV and bundled CSV/Parquet forms in addition to existing exchange formats. Treat this as an interoperability experiment only; do not make an unfinalized pre-release a mandatory production dependency.

Candidate acceptance tests for an OCEL adapter:
- object identity preserved across episode, complaint, asset, zone, work order, crew, permit and sample relations;
- event-to-object role qualifiers preserved (for example, responding crew, affected zone, sampled source);
- object-to-object relations preserved (for example, work order repairs asset; sample tests water source);
- object attribute changes and valid-time/recorded-time distinctions round-trip without silently overwriting history;
- duplicate/retried events remain distinguishable from distinct repeat service episodes;
- corrections/revocations remain explicit event lineage, not destructive in-place edits;
- unknown source fields survive in an extension namespace or produce an explicit lossy-export receipt;
- round-trip output is deterministic and lossless for the declared profile.

OCEL is an interchange/process-mining representation, not a source-authenticity, evidence-independence or authority model. Keep Mycelix owner packets and receipts as the semantic assurance layer; do not attempt to encode their trust semantics as unqualified OCEL attributes.

References:
- OCEL 2.0: https://www.ocel-standard.org/
- OCEL 2.1 pre5 specification: https://www.ocel-standard.org/2.1/ocel20_specification.pdf
- OCEL 2.1 CSV format: https://www.ocel-standard.org/specification/formats/csv/
- OCEL dataset/publication collection: https://ocel-standard.org/event-logs/publications/

## 4.1 Restricted OCEL 2.0 profile

The interoperability branch includes one synthetic OCEL 2.0 JSON profile at `ocel/ocel-water-service-v1.json` and a Rust validator/canonicalizer at `harness/src/ocel_profile.rs`.

The profile's 17 objects and 12 events cover complaints, same-fault callout grouping, service episodes, repeat episodes, zones, assets, work orders, crews, permits, samples, sources, lab results, pressure observations and decision revisions, plus one source-assertion correction and one revocation. Event attributes carry source-recorded time and a host-assigned append sequence. Objects carry time-stamped attribute changes and qualified object-to-object relationships.

The Rust profile validator checks:
- object/event type declarations and typed attributes;
- event and object identifier uniqueness;
- event/object type references and qualified relationship targets;
- RFC3339-style timestamps with explicit timezone;
- attribute identity uniqueness (event attribute name; object attribute name plus time);
- event-to-object and object-to-object references;
- deterministic canonical ordering and a SHA-256 receipt.

It preserves unrecognized JSON extension values through canonicalization. The canonical digest proves only reproducibility of the declared JSON representation; it is not a source signature, an evidence-authenticity claim, an owner-core result or a semantic service closure.

The profile is deliberately restricted. Independent importer/exporter round-trips remain required before claiming general OCEL compatibility. The current official OCEL 2.0 JSON documentation defines the four collections and allowed value types, but the JSON-schema URL listed in the 2.0 specification PDF returned HTTP 404 during the 2026-11-27 research pass. The `schemas/schema.json` file in the `OCEL-standard/ocel-support` repository uses legacy `ocel:events` / `ocel:objects` keys and is not an OCEL 2.0 JSON schema. Do not treat that legacy schema as a validator for the modern array-based OCEL 2.0 format. A schema gate must use a verified, versioned OCEL 2.0 schema source or a separately reviewed schema derived from the normative specification, with source URL, commit and digest recorded. OCEL 2.1.0pre5 serialization is not implemented and remains a separate experimental target.

### Differential interoperability benchmark (next gate)

The independent oracle must not share the Rust profile validator's parser or canonicalizer. The official OCEL tool-support matrix lists PM4Py, FLOWVAULT and Ocelot.pm as OCEL 2.0 JSON-capable implementations. PM4Py documents explicit OCEL 2.0 JSON read/write operations; Rust4PM documents JSON import/export through Polars. Ocelot is useful for human inspection, but a browser inspection alone is not a reproducible CI oracle. An exploratory PM4Py 2.7.23.15 importer/exporter round-trip script now exists at `oracles/pm4py_ocel2_roundtrip.py`. It compares a normalized OCEL inventory and is explicitly test-only. Its direct package version is pinned in-script, but the transitive dependency graph and wheel hashes are not yet locked; no PM4Py round-trip pass is claimed until that environment is frozen and executed.

Reference implementations:
- https://www.ocel-standard.org/tool-support/overview/
- https://www.ocel-standard.org/tool-support/libraries/pm4py/
- https://github.com/process-intelligence-solutions/pm4py/blob/release/docs/source/api.rst
- https://rust4pm.aarkue.eu/docs/python-api/

Keep the application and production adapters Rust-first. A pinned external importer/exporter may be used strictly as an independent test oracle in CI. It must emit a normalized inventory (object IDs/types, event IDs/types/timestamps, typed attributes, E2O/O2O relationships and qualifiers, object-attribute changes) for comparison with the Rust profile receipt. Do not compare only counts or byte hashes.

Required differential cases:
1. Rust export -> independent import -> normalized inventory comparison.
2. Independent export -> Rust import -> normalized inventory comparison.
3. Reordered arrays and object keys -> semantic equality.
4. Unknown extension fields -> preserved or explicit lossy-export receipt.
5. Invalid references, duplicate identities and malformed timestamps -> deterministic rejection or documented importer-specific refusal.
6. Same-fault complaint grouping and distinct repeat episodes -> identity and relationship preservation.
7. Late arrival, correction, revocation and two-frontier as-known-at cutoff -> no history rewrite.
8. Reconstruct the same valid-time slice at accepted-sequence frontiers 10, 11 and 12; event 006 remains effective at frontier 10, is superseded at 11, and event 008 is revoked only at 12.
9. Reordering the OCEL event array cannot change the replay receipt; duplicate host sequences, unknown mutation targets, non-causal mutations and conflicting mutations fail closed.

### Declarative object-centric process comparator (research target)

A second relevant baseline appeared on 24 November 2026: *Discovery and conformance checking of object-centric DCR graphs from event knowledge graphs*. It formalizes Object-Centric Dynamic Condition Response (OC-DCR), adds alignment-based conformance over event knowledge graphs, evaluates rediscoverability on synthetic multi-object logs, demonstrates discovery on BPIC17 and the OCEL Order Management log, and benchmarks alignments under eight injected-noise levels. Its synthetic rediscoverability experiment uses 1,000 simulated cases and includes one-to-many and many-to-many synchronization. The paper reports rapidly increasing alignment runtime as alignment cost rises, with its benchmark imposing a five-minute timeout per alignment; its analysis also notes that global synchronization can be too coarse for some object-centric processes.

This gives Craft a more exact comparator than generic workflow engines:
- OC-DCR / OCDisCoveR: object-centric declarative lifecycle constraints, spawning and multi-object synchronization, discovery and alignment-based conformance.
- PM4Py / FLOWVAULT: event-log interoperability, object-centric process discovery and lifecycle analytics.
- Mycelix Craft: owner-specific evidence obligations, freshness/revocation propagation, authority scope, physical-outcome distinctions and conclusion-relative closure.

Do not compete with OC-DCR on model discovery unless the same benchmark corpus and metrics are implemented. Instead add a matched assurance layer around the same traces and objects. For example, OC-DCR may establish that a work-order/permit/crew sequence conforms to a declared process model; Craft must separately establish whether the permit source is authentic/current, whether the assigned crew qualification was valid at the relevant valid-time and knowledge frontier, and whether independent evidence supports physical restoration. A process-conformance pass is not an evidence-authenticity or service-outcome pass.

Add a comparator suite with:
1. Synthetic ground-truth rediscoverability across one-to-many and many-to-many object synchronization.
2. BPIC17 and OCEL Order Management baseline import/discovery/conformance to verify toolchain execution.
3. WSR domain-specific OC-DCR constraints for permit-before-work, qualified-crew assignment, sampling-before-result, pressure observation, independent quality support and decision closure.
4. The same eight event-noise families used in the published OC-DCR evaluation (event deletion, insertion and same-trace swaps) plus WSR-specific faults: wrong-zone relation, stale permit, revoked qualification, mirrored source, correction, revocation, late arrival and forged/unknown source.
5. Separate event-log conformance score, alignment cost/runtime, timeout rate, owner-core disposition agreement, evidence-lineage invalidation recall and false-safe rate.

Report all alignment timeouts, not only completed alignments; stratify by trace length, object count, synchronization fanout, alignment cost and noise rate. Any five-minute timeout is an uncompleted observation, never a pass. Include a separately adjudicated oracle for evidence truth and authority: OC-DCR conformance alone cannot validate those properties.

Reference:
- OC-DCR discovery and conformance paper (published 2026-11-24): https://link.springer.com/article/10.1007/s44311-026-00061-9
- OCEL 2.0 process-mining tool support: https://www.ocel-standard.org/tool-support/overview/

### Published OCPM scale anchors (context, not a head-to-head result)

A 2026 study, *Advancing object-centric process mining with multi-dimensional data operations*, evaluates four publicly available BPIC-derived OCEL collections (BPIC 2014, 2016, 2017 and 2019). The reported logs span roughly 0.7 million to more than 7 million events and around 0.1 million to more than 0.7 million objects. Its BPIC 2016 stress case reports about 29 seconds per unfold call and about 60 seconds for drill-down; loading that log into PM4Py was roughly an order of magnitude slower than loading the other studied logs. The paper also reports that the OCPA conformance implementation timed out on 20 of 91 groups during fitness/precision calculation, despite the log transformations being applicable to all 91 groups.

These results are **published context, not a directly comparable baseline**: benchmark hardware, versions, preprocessing, and detailed runner conditions must be matched before drawing performance conclusions. They do, however, make the 10^7-event target concrete rather than arbitrary and highlight two important gaps to benchmark: expensive import/whole-log materialization and conformance under complex object-centric parallelism.

Add these public workloads to the benchmark registry:
- BPIC 2014 / 2016 / 2017 / 2019 OCEL variants for parse, canonicalize, relation inventory, temporal projection and scale testing;
- public object-centric state-aware logs for state-transition and dynamic object-relationship reconstruction;
- the WSR synthetic corpus for domain-specific correction, revocation, duplicate-callout and safety/closure semantics.

For process analysis, compare the data transformations against the published drill-down, roll-up, unfold and fold operations and state-aware lifecycle exploration (including FLOWVAULT). Mycelix does not currently implement all four OLAP transformations or a general process-discovery engine; those are separate comparator tasks, not implied capabilities. The novel target is to preserve evidence lineage and reproduce decision-relevant state at exact historical frontiers while still interoperating with those analytical views.

References:
- 2026 OCPM multi-dimensional operations and BPIC scale study: https://link.springer.com/article/10.1007/s10115-026-02904-0
- Official OCEL event-log collection: https://ocel-standard.org/event-logs/overview/
- FLOWVAULT state-aware OCEL lifecycle analysis: https://ocel-standard.org/tool-support/software/flowvault/

Scale ladder (report each result separately, do not average correctness with speed):
- 10^3 events: developer feedback and mutation-test corpus.
- 10^5 events: practical integration scale.
- 10^7 events: stress/streaming scale, if memory budget and runner permit.

Metrics: parse/export throughput; peak RSS; serialized bytes; normalized preservation by field/relation category; event/object identity precision and recall; qualifier preservation; temporal-frontier reconstruction; correction-history preservation; deterministic digest across input permutations; and owner-disposition agreement before/after interchange. Publish p50/p95 runtime and peak memory, exact tool versions, hardware/runner class, corpus digest and command line.

A conventional case-centric XES flattening baseline is useful only as a measured loss comparator. It must not be treated as an equivalent object-centric representation or as a correctness oracle. Public BPI/OCEL process-mining logs can benchmark generic discovery/conformance throughput, but they do not validate municipal water-service semantics; the WSR synthetic adversarial corpus remains the semantic benchmark.

Targets (not results): zero silent field/relation loss within the declared profile, 100% normalized round-trip preservation, zero duplicate-callout inflation, zero repeat-episode merges, and unchanged owner-core dispositions after serialization round-trip.

### Temporal relationship-claim projection (prototype slice; exact-head qualification pending)

The experimental Rust module `harness/src/temporal_relationships.rs` models a relationship as a provenance-carrying claim rather than a timeless edge. Fields include relation ID, subject/predicate/object, half-open valid interval `[valid_from, valid_to)`, source-recorded time, accepted host sequence, source reference/digest, asserting principal and authority scope. Append-only mutations supersede or revoke a relation. Supersession points to a replacement claim and includes a digest commitment to the exact target claim.

At query time, the projector applies two independent cutoffs: valid time and accepted sequence. It returns visible relation IDs, active claims, conflicting subject/predicate assignments, applied mutation IDs, superseded/revoked IDs and a deterministic snapshot receipt. Competing active assertions are surfaced as conflicts; no last-write-wins selection occurs. Mutation records are validated in accepted-sequence order. A supersession chain is permitted only when the replacement has already been accepted, retains the target's subject/predicate/authority scope, and is not a previously retired relation.

The prototype currently has focused unit tests for late-recorded relation visibility, conflicting asset-zone assignment, supersession, revocation at both frontiers, source-claim digest binding, authority-scope mismatch, duplicate accepted sequence, half-open expiry, supersession chains, attempted relation reactivation and input-order invariance.

This is an in-memory typed projection, not yet a JSON parser or direct OCEL-to-tEKG adapter. It carries source digests but does not authenticate source bytes, verify signer identity, prove the authority scope is valid, or establish the relationship's truth. Equality of authority-scope strings is a structural boundary only, not an authorization decision. Host sequence is a knowledge-order frontier, not trusted transaction time. Exact-head compilation/formatting/Clippy and independent known-answer oracle results are pending.

### Dual-frontier temporal replay (implemented slice)

The Rust harness exposes `temporal_replay::replay_temporal_frontier`. It evaluates two inclusive cutoffs: (1) event valid time, represented by canonical fixed-width UTC seconds in this V1 replay path, and (2) a host-assigned append sequence representing what had been accepted into the log. The host sequence is an ordering frontier, **not** a timestamp and not proof of when a platform committed or independently verified an event. The source-recorded timestamp is validated and preserved but is not promoted to transaction time.

The fixture now has 12 events. Event-011's valid time is 11:45Z while its source-recorded time is 13:00:04Z and its host sequence is 11, deliberately modeling a correction recorded after the time it corrects. At valid-time cutoff 12:00Z, accepted frontier 10 still exposes event-006; frontier 11 applies event-011 and supersedes event-006. At frontier 12, event-012 revokes event-008. A valid-time cutoff before 11:45Z excludes event-011 even when the accepted-sequence frontier includes it. Mutations must share at least one object identity with their target and V1 only allows them to target original assertion events; unrelated-object mutation attempts fail closed. The receipt reports visible events, effective event IDs, superseded IDs, revoked IDs, applied mutation IDs and a deterministic SHA-256 replay digest.

This is a deliberately bounded dual-frontier replay primitive, not a full bitemporal database: no trusted platform transaction-time clock, source signature, authority proof, causal truth guarantee, or domain-level correction policy is asserted. V1 requires UTC second precision for temporal ordering; general RFC3339 validation remains in the OCEL profile layer. Later work should add explicit platform-accepted-at metadata, source-specific correction authority, correction chains, and event-time/knowledge-time uncertainty intervals before claiming general bitemporal support.

Replay correctness targets (not results): zero history rewrites across knowledge frontiers; 100% agreement with a separately authored cut/query oracle; deterministic receipts under event-array permutation; and fail-closed rejection of duplicate sequences, missing targets, forward references and conflicting mutations. Report replay throughput and memory separately from semantic correctness.

### Rust scale probe (implemented; no results claimed)

`harness/src/bin/ocel_scale_bench.rs` generates a deterministic synthetic OCEL JSON log by writing events incrementally, with one Complaint object per event and a configurable complaints-per-Callout fan-out. It scales event and object cardinality together and allows the relation fan-out to vary while holding event count fixed. It then measures the existing whole-document Rust validator/canonicalizer and the dual-frontier replay path separately. It performs two warm-up passes, then reports input and canonical byte counts, input/canonical/replay digests, p50/p95 validation-plus-canonicalization latency, p50/p95 replay latency, event throughput for both paths, replay-visible/effective counts, OS/architecture and Linux peak RSS where available. The CLI requires 5–100 measured repeats and defaults to 20. It is a reproducible measurement harness, not a performance claim and not a streaming parser. The present validator loads the whole JSON document and materializes canonical output; this probe is intended to expose that memory/latency curve before a separate streaming architecture is proposed.

Run in the materialized workspace using `cargo +1.96.0 run --release -p craft-water-cross-core-harness --bin ocel-scale-bench -- 100000 /tmp/ocel-100k.json 20 100`. The final argument is complaints-per-callout (default 100); vary it across 1, 10, 100 and 1,000 to exercise different object counts and edge fan-outs. Benchmark 10^3, 10^5 and 10^6 events on a fixed runner; reserve 10^7 for a suitably provisioned isolated host. Record exact source SHA, toolchain, CPU/RAM/OS, workload digest, command, raw JSON output and repeated-run statistics. The executable's compile/Clippy qualification and benchmark measurements are pending on this branch.

## 4. Frozen corpus and executable coverage

The source fixture bundle contains 16 known-answer synthetic WSR episodes and 20 proposed metamorphic transformations.

Current executable slice:
- WSR-001: provider reports completion; current pressure restoration and water-quality evidence are absent. Must remain NotEligible; Decision Closure remains OpenMissingElements.
- WSR-004: current pressure and water-quality evidence is supported. The episode may reach EligibleForLocalReview, while one required material-element mapping review remains absent; Decision Closure returns MappingReviewRequired and never Closed.
- WSR-006: a wrong-zone pressure observation has a contradictory Scope qualifier; Claim Support returns HumanReviewRequired rather than Supported.
- Source expiry propagates NeedsRevalidation to dependent claim and decision artifacts while an unrelated quality source remains Current.
- WSR-009: an expired permit and revoked crew qualification produce a safety gate block and targeted work-authorization revalidation.
- Two observations sharing one independence group do not satisfy a two-independent-source rule.

The six tests call all six public owner evaluators. The service-level oracle is separately authored and does not call the owner evaluators. This slice is not the full WSR benchmark: four of the 16 known-answer cases are implemented; the remaining 12 cases and most other metamorphic relations are not yet run through the adapters.

## 4. Correctness gates

Correctness is a hard gate. A single high-consequence false-safe outcome blocks qualification.

Proposed gates (targets, not achieved results):
1. 16/16 exact known-answer dispositions against an independently adjudicated oracle.
2. 20/20 registered high-risk metamorphic transformations produce the expected change or preserve-the-same result.
3. Zero provider-reported-complete-only cases treated as physical restoration or closure.
4. Zero stale, expired, revoked, wrong-zone, wrong-context, wrong-jurisdiction or wrong-revision evidence accepted as current support.
5. Zero false NotAffected / false independent classifications after a relevant high-consequence mutation.
6. Zero mirrored or common-cause evidence counted as independent corroboration.
7. Zero denominator expansion, shrinking or substitution after authorization.
8. 100% targeted revalidation recall for high-consequence changes on the frozen oracle; precision and false holds reported separately.
9. 100% deterministic replay of exact frozen inputs, toolchain, owner revisions and adapter build.
10. Zero auto-closed service episodes. “Eligible for local review” is the ceiling of this reference slice, not a permission to execute work or release water.

For every mutation, record exact field-level transformation, source and target packet digests, risk tier, owner outputs, expected oracle output, whether the mutation is applicable, and a classification of detected / equivalent / invalid / survived. Do not report a mutation score until all registered mutations were executed.

## 5. Outcome metrics (separate from evaluator correctness)

Report outcomes independently; do not collapse them into one weighted score:
- customer minutes off supply and interruption duration;
- interruption count and repeat-failure rate;
- hours of supply per day / continuity;
- pressure-deficit exposure by zone and duration;
- water-quality sample pass rate and monitoring coverage;
- response time, repair time and verified restoration time as separate clocks;
- alternative-supply volume, duration and adequacy;
- customer complaints and water-quality complaints;
- distributional impact by relevant service area/population denominator;
- worker safety events, near misses and qualification/permit gate failures.

The exact operational definitions, sample sizes, time windows, missing-data policy and denominator are part of each benchmark revision. Averages must not hide prolonged outages, repeat bursts, localized water-quality risks or unequal impacts. Industry water-utility indicator frameworks track continuity, interruption count/duration, pressure, water-quality monitoring and response/restoration time; reuse their definitions only where units and populations match.

## 6. Performance and resource metrics

Measure only after correctness passes:
- per-owner evaluation latency and end-to-end cross-core latency p50/p95/p99;
- packet construction, canonicalization, digest and receipt-generation time;
- wall time and CPU per mutation;
- peak memory and bytes per episode / per owner receipt;
- dependency graph size, change fan-out and lineage invalidation cost;
- number of owners evaluated, proofs/receipts examined, and reasons for requalification;
- human-review queue volume, time-to-review and false-hold rate;
- cold/warm throughput under fixed concurrency;
- incremental re-evaluation cost against full recomputation.

Freeze the CPU, RAM, OS/kernel, toolchain, lockfile, source commit SHAs, case digests, evaluator/adapter revisions, concurrency, warm-up, and measurement seed. Publish raw machine-readable results and run medians/p95, not only a headline average.

## 7. SOTA comparison strategy

A credible improvement is not simply faster workflow automation. Mycelix's differentiator must be measured as *decision quality under incomplete, corrected, correlated and adversarial evidence*.

Compare:
- ordinary CMMS/ITSM status closure;
- event-sourcing / replay baseline;
- process-mining extraction and conformance baseline;
- human-adjudicated source packet;
- Mycelix composed owner evaluators.

For each, publish a capability matrix stating whether it can express:
- exact scope and immutable source revisions;
- distinction between provider report and independently observed restoration;
- explicit uncertainty / abstention;
- correction and revocation lineage;
- common-cause independence;
- immutable authorized denominator;
- safety-critical capability and permit blockers;
- historical replay under the same accepted evidence frontier;
- independently reviewable conclusion mappings.

If a comparator cannot express one of these, mark it “not represented” and test the nearest native behavior. Never claim it failed the semantic test if its interface has no such claim.

## 8. Field-pilot boundaries

All current fixtures are synthetic. A real municipal pilot requires appropriate authority and community approval, privacy/data-sharing agreements, worker-safety controls, operational and water-quality criteria approved by qualified professionals, independent evaluation, and an explicit rollback/incident process.

A synthetic correctness pass is not legal compliance, water-quality certification, field readiness, or causal evidence that service outcomes improved.
