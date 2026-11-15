# Scientific observability benchmark protocol v0.1

Status: proposed protocol. No system leaderboard or real-science performance claim is made by this document.

## 1. Benchmark the whole decision system

The unit under test is not just a language model or anomaly detector. Record the complete configuration:

- model and model version;
- reasoning agent and policy;
- parsers, source adapters and evidence stores;
- benchmark harness and tool versions;
- system prompts/configuration and safety/authority policy;
- compute, wall-clock, data, reviewer and instrument budgets.

A run is not reproducible if any of these are omitted. Freeze the task inputs, split definitions, budget, seeds, expected evidence graph and success criteria before comparing systems.

## 1A. Domain-native space-time preservation

The VOEvent parser/replay now has an initial source-native WhereWhen extraction slice under [ASTRO-WHEREWHEN-001](https://github.com/Luminous-Dynamics/mycelix/issues/6209). The canonical parser captures the complete \`WhereWhen\` subtree into an ordered source-element tree and the lifecycle replay carries that tree into its visible packet snapshot. It preserves element and attribute order, text, CDATA, character-reference boundaries, prefixed child names, unknown STC extensions, units, UCDs and uncertainty elements. The tree remains bound to the original packet digest.

This is a **source-preserving extraction profile**, not a complete STC model, XSD validator or coordinate transformation engine. It deliberately leaves \`event_time\` absent and does not normalize UTC/TT/GPS/TDB, ICRS/FK5/solar frames, TOPO/GEO/BARY, units or uncertainty. That is safer than a universal RA/Dec/UTC shortcut. See [IVOA VOEvent 2.1 Recommendation §3.4](https://www.ivoa.net/documents/VOEvent/20260429/REC-VOEvent-2.1.html).

Frozen WhereWhen/STC cases should include UTC-ICRS-TOPO, TT-FK5-GEO, GPS and TDB-ICRS-BARY; Position2D, Position3D and PositionName; TimeInstant and TimeInterval; observatory and observation locations; coordinate units, UCDs, axis names, radius and per-axis uncertainties; custom TimeFrame/SpaceFrame definitions; missing locators; contradictory coord_system_id references; XML namespace variation; and unsupported solar-system frames.

Hard gates:
- zero silent time-scale, reference-frame, reference-position or unit conversion;
- zero loss of original coordinate/time lexemes, element/attribute order, uncertainty, axis semantics or observatory identity in the source tree;
- no default zero-valued coordinates or timestamps when a locator is absent;
- unsupported structures remain byte-bound and explicitly unsupported;
- any later conversion must have a separate transform receipt identifying input/output systems, algorithm/library/version, constants/ephemeris, uncertainty propagation and output digest;
- historical replay preserves the source-native packet projection even when a new transform is added later.

Current executable controls cover UTC-ICRS-TOPO with TimeInstant and Position2D/errors, TT-FK5-GEO and TDB-ICRS-BARY lexical frame IDs, PositionName, TimeInterval, units/UCDs, an unknown extension, a prefixed WhereWhen subtree and source-tree propagation through replay. Position3D, GPS-specific fixtures, custom TimeFrame/SpaceFrame, contradictory frame references, missing-locator permutations and complete XSD/STC conformance remain unimplemented test cases—not completed support.

Do not score a parser as more capable merely because it converts more fields. A conversion with hidden assumptions is worse than an explicit unsupported state.

## 2. Evaluation ladder

Keep the same task API and evidence semantics while increasing physical realism.

| Tier | Evaluation environment | What it establishes | What it does not establish |
|---|---|---|---|
| T0 | Synthetic, adversarial, known-answer corpus | Parser/replay semantics, temporal isolation, exact graph preservation, deterministic behavior | Real-world scientific validity |
| T1 | Retrospective, rights-cleared public observations | Reproducibility against historical data; forecast and investigation quality | Prospective operational value |
| T2 | Cross-domain and out-of-distribution holdouts | Transfer across observatories, sensors, platforms, geography, season, processing and annotation | Robustness to all unseen environments |
| T3 | Digital twin with latency, noise, failures and common-cause faults | Closed-loop decisions under controlled physical-model uncertainty | Real instrument performance |
| T4 | Authorized live instrument or field campaign | Prospective scientific and operational value | Universal generalization beyond the campaign |

Do not move to T4 on the basis of a benchmark score alone. It requires separate rights, safety, operator and instrument authorization.

## 3. Baseline tournament

Every task should compare the full system against the following baselines under identical initial evidence and resource budgets:

1. No follow-up.
2. Fixed-schedule follow-up.
3. Existing expert or broker workflow.
4. Threshold/rule-based alerting.
5. Anomaly-score greedy selection.
6. Entropy or information-gain greedy selection.
7. Decision-theoretic EVSI/ENBS selection.
8. Model-only and agent/tool-enabled variants.
9. Symthaea as the complete configured system.
10. Hidden-truth oracle on synthetic data only, as an upper bound rather than a deployable competitor.

For decision policies, match follow-up count, instrument time, exposure, latency and monetary/compute budget. For model/agent systems, record the exact model, agent, harness, prompt/configuration and tool set.

## 4. Scorecard

Never collapse all dimensions into a single accuracy number. Publish a per-domain scorecard.

| Dimension | Primary measures |
|---|---|
| Prediction | Brier score, log loss, reliability/calibration, precision-recall, interval coverage |
| Event detection | False alarms per unit exposure/time, missed-event rate, detection latency |
| Scientific value | Realized decision loss/regret, accepted discoveries, hypothesis discrimination, time-to-resolution |
| Follow-up utility | EVSI/ENBS, information gained that changes a decision, follow-up cost |
| Evidence integrity | Temporal leakage, packet/digest binding, lost citations, unresolved-reference handling, replay determinism |
| Generalization | Leave-one-source/domain/region/sensor/platform/season/processing/annotation-out results; worst-stratum performance |
| Human work | Reviewer minutes, escalations, reversals, evidence reconstruction time |
| Resource efficiency | Compute, wall time, data transfer, instrument time and cost per accepted resolution |
| Governance | Rights violations, unauthorized actions, missing authority, provenance bypasses |

Hard gates are not averaged into the score. Any temporal look-ahead, silent history deletion, digest mismatch accepted as valid, rights bypass or unauthorized instrument action is a gate failure.

## 5. Required adversarial cases

At minimum, freeze known-answer cases for:

- late-arriving evidence with earlier source publication time;
- source event time that is not UTC or is not directly comparable;
- duplicate, missing, malformed and reused identifiers;
- unusual or uninterpreted root/citation attributes and exact description whitespace, including unresolved citation targets;
- follow-up, supersession and retraction by different publishers;
- unresolved references that become resolvable later;
- contradictory and cyclic citation graphs;
- packet alteration, digest mismatch, parser/projection disagreement, caller-backdated receipt assertions, missing/pending/revoked authority receipts and incomplete receipt-chain records;
- shared source, gateway, operator, calibration or trust-root common causes;
- clock inversion, stale calibration, telemetry-only evidence and source conflict;
- domain shift, missing modalities, sensor failure and annotation disagreement;
- a genuinely unknown or misspecified physical model, where OTHER/abstain is correct.

A negative control must fail closed. A system must not receive credit for a correct answer reached by reading data that would not have existed at the decision cutoff.

## 6. Statistical protocol

- Pre-register task versions, data rights, splits, metrics, baselines, budgets and exclusion rules.
- Use event/mission/time-blocked splits; never randomly split correlated frames or packets across train and test.
- Hide final test labels and outcomes where feasible.
- Use paired comparisons on the same tasks and seeds.
- Report distributions and confidence intervals, not only means or leaderboard ranks.
- Use cluster-aware bootstrap or a justified hierarchical model when observations share an event, source, site or campaign.
- Determine trial counts by power analysis. Three trials per task are a pilot, not a robust ranking.
- Publish per-domain and worst-stratum results. A large domain must not hide failure in a smaller one.
- Include failed, abstained and unresolved trials in accounting; do not silently drop them.
- Publish immutable task manifests, exact input digests, source licenses/rights, evaluator version, run receipts and full configuration.

## 7. Proposed acceptance targets

These are goals for future evaluations, not results achieved by v0.1.

### Integrity gates

- Zero temporal look-ahead across all known-answer and adversarial cases.
- Every replay reports the provenance of its cutoff; caller-supplied fixture timestamps are explicitly `unattested_manifest_assertion` and cannot qualify a production as-known-at claim.
- Zero source-projection mismatch: packet identity, role, publisher assertion, creation time, citations and digest must originate from one parser invocation over the same exact source bytes.
- Zero loss of source root attributes, EventIVORN attributes or Citations/Description text in the replay graph, including unresolved edges.
- Zero silently lost source packets or lifecycle citations.
- 100% expected edge/effect fidelity on the frozen synthetic corpus.
- 100% deterministic replay for identical inputs and cutoff.
- Zero silent coercion of conflicting lifecycle assertions into a single canonical truth.
- Zero rights, authority or instrument-tasking bypasses.

### Scientific-value goals

- At least 20% lower held-out investigation regret than the strongest non-oracle baseline at the same follow-up and compute budget is a stretch target.
- No material degradation in the worst held-out domain while improving pooled performance.
- Calibrated uncertainty and explicit abstention/OTHER when the evidence cannot distinguish hypotheses.
- Lower reviewer and follow-up burden at equal or better scientific decision quality.

The 20% regret target is not a claim of current superiority. It must be tested with paired, held-out tasks and confidence intervals.

### Research-frontier stretch targets

These are intentionally ambitious, proposed targets, not achieved results:

- **EarthShift-style geospatial OOD:** after faithfully reproducing the published baselines on the same 11 tasks and five shift families, target an average OOD degradation of **10% or less**, compared with the paper's reported 15–20% average degradation, while limiting in-domain regression to at most 1 percentage point and improving the weakest geography/scale/sensor/source shift. Report all five axes separately; a pooled gain cannot hide a worsening sensor or geographic shift.
- **Full-cycle scientific rediscovery:** target **greater than 60 F1** on a FIRE-Bench-compatible evaluation under matched task, model, tool, compute and execution budgets, against the paper's reported frontier results below 50 F1. Require at least five independent runs per task or a power-justified larger sample, task-level confidence intervals, and independent verification of the reproduced finding. This is a research-agent benchmark target, not direct proof of success in astronomy, Earth science or field campaigns.
- **Scientific ideation:** improve evidence-grounded novelty against literature retrieval while preserving or improving falsifiability and feasibility. Report the originality, grounding, clarity, specificity and feasibility critics independently; do not optimize a single blended LLM-judge reward.
- **Decision quality:** preserve the ≥20% held-out regret-reduction stretch target at the same eligible follow-up/compute budget, with hard integrity and rights gates applied before scoring.

A result only counts as a SOTA comparison after the reference protocol, dataset rights, split, model/harness configuration, adaptation budget and evaluator have been reproduced. An unavailable or non-equivalent baseline is `NOT_RUN`, not a loss or win.

### Scale gates

Measure replay at 10^3, 10^4, 10^5 and 10^6 packet records, reporting throughput, memory, p50/p95 latency and index/build costs. Keep correctness tests enabled at each scale. Do not optimize away provenance or temporal semantics to improve throughput.

## 8. Position relative to current benchmarks and standards

Autonomous Science Bench 0.1 is a useful methodological reference for full-configuration evaluation, task-specific discovery and learning outcomes, equal-budget baselines, and the replay-oracle → digital-twin → live-lab progression. Its initial public release has six pilot tasks, three trials per task, and explicitly warns that small leaderboard gaps are not meaningful. Its percentages are not comparable to an astronomy event-reasoning benchmark.

CrossCD-Eval (Scientific Reports, 3 October 2026) evaluates remote-sensing change detection across in-domain, cross-domain and few-shot settings using CLCD, HRSCD and LEVIR-CD. Its findings reinforce that strong in-domain results do not guarantee cross-domain transfer. Our satellite evaluation should therefore hold out geography, sensor/platform, season, resolution, processing and annotation regimes independently.

FIRE-Bench (ICML 2026, PMLR 306) evaluates full-cycle rediscovery of documented research findings through experiment design, execution and evidence-backed conclusions. Its authors report that even the strongest tested frontier agents remained below 50 F1, with high run-to-run variance and recurring failures in design, execution and evidence-based reasoning. This supports repeated trials, exact evaluation receipts and outcome verification rather than paper-style LLM judging.

AgentIdeaBench (September 2026) compares static observation with active exploration across 33 models, 40 subfields and five disciplines. Active exploration improved grounding, feasibility, clarity and specificity, while measured originality did not improve under its critics. Our scientific-investigation scorecard must therefore separate novelty/originality from falsifiability, grounding, feasibility, experimental execution and verified outcome. An idea that is practical but not novel, or novel but not testable, must not be collapsed into one quality score.

The 17 October 2026 lifecycle survey of autonomous scientific-research agents analysed 253 studies in depth and concludes that the field has matured unevenly: retrieval-grounded research and ML engineering show repeatable gains, while open-ended ideation, end-to-end discovery and trustworthy reporting remain fragile. Its reported benchmark outcomes range from 16.9% Kaggle bronze-level competition success to 21.0% best reproduction across 20 papers, 32.4% scientific-coding tasks solved and 31.5% memorization-resistant equation discovery; an o1 agent scored 26.6% on a three-paper replication subset versus 41.4% for ML PhDs. These are different tasks and should **not** be pooled into one “scientific-agent SOTA” number. Use them as maturity and difficulty context, then compare our own matched task families with identical budgets, pre-registered outcomes and verified run receipts.

Source: [Gao, Yu & Li, “Large Language Model Agents for Autonomous Scientific Research: A Lifecycle Survey of Methods, Systems, Benchmarks, and Risks,” Journal of Superintelligence, 17 October 2026](https://www.mdpi.com/3043-0097/1/2/8).

EarthShift (arXiv:2605.29330, 28 May 2026) is a particularly relevant geospatial OOD comparator. It evaluates 8 geospatial foundation models across 11 tasks and five natural distribution-shift families: spatial scale, temporal, geographic, sensor/modality and data-source shift. Its reported OOD performance is 15–20% lower on average. It found temporal shift less damaging than geographic, scale, sensor and source shifts; domain-specific pretraining did not create a consistent robustness advantage over generic vision or fully supervised baselines. It also includes a South Africa season-to-season agricultural segmentation task.

For the Earth-observation adapter and downstream models, add a matched EarthShift-style track:
- reproduce the published ID/OOD splits where dataset rights and access permit; do not assume public discoverability grants redistribution rights;
- compare at least one classical supervised baseline, a generic vision foundation baseline and multiple geospatial foundation baselines;
- report ID score, OOD score, absolute degradation and effective robustness relative to the fitted ID→OOD baseline, separately for all five shift families;
- use geography, scale, sensor/modality and source shifts as primary stress tests; include temporal/seasonal controls;
- report worst-shift and worst-region performance, uncertainty calibration, abstention and adaptation budget; never publish only the mean;
- keep scene, region, campaign and acquisition clusters disjoint across splits to prevent leakage.

Add an explicit **shifted-calibration track**, not just confidence-ranked accuracy. Lehmann et al. (17 August 2026) report that across 16 frozen geospatial encoders and four classification/five segmentation datasets, corruption degrades all encoders; under shift, GeoFM encoders become more overconfident than ImageNet-pretrained encoders. Their tested temperature-scaling and deep-ensemble methods did not correct this degradation, while one Gaussian-process probe improved severe-cloud ECE at the cost of much worse clean ECE. Confidence-based abstention also failed to defer confidently wrong predictions.

Therefore require, per shift family and severity:
- Brier score, negative log-likelihood and classwise/adaptive ECE with reliability diagrams;
- calibration intercept/slope and risk-coverage/AURC, including confidently wrong cases;
- selective risk at fixed coverage and coverage at fixed risk, stratified by class/region/sensor;
- performance when the model is allowed to abstain or return OTHER, with abstention cost counted;
- calibration transfer tests: source-only calibration, small target calibration budget, and no target labels;
- paired uncertainty estimates and bootstrap intervals, with repeated runs and worst-shift results;
- separate reporting for post-hoc temperature scaling, ensembles, GP/structured uncertainty, and model-native uncertainty.

A confidence method that improves ECE on severe corruption but damages clean calibration or fails on another shift must be reported as a trade-off, not a universal improvement. Optimizing average confidence calibration alone is not enough: the target is lower decision risk under shift with an explicit, auditable abstention policy.

EarthShift is an external comparator, not a claim that our adapter or detector currently matches or exceeds it. Our contribution should be evidence-preserving replay, source/rights-aware cross-domain provenance, common-cause controls and decision-grade investigation under OOD—not a new in-domain F1 record alone.

IVOA VOEvent 2.1 already standardizes event packets and lifecycle citations. This work should interoperate with that standard, not invent a competing astronomy packet format. The additional contribution is an auditable, receipt-time-aware evidence and decision layer around source-native packets. VOEvent itself does not define the transport layer, publisher policy or intellectual-property/rights policy; those remain explicit system responsibilities.

## 9. Industry comparison: integrate with alert brokers, do not clone them

Rubin Observatory documents seven full-stream brokers (ALeRCE, AMPEL, ANTARES, Babamul, Fink, Lasair and Pitt-Google) plus downstream services such as SNAPS and POI Broker. Their documented functionality already includes filtering, cross-matching, contextual enrichment, photometric classification, user-defined filters, prioritization, watchlists, APIs and stream interfaces. Examples include ALeRCE's aggregated/classified light-curve catalogue, ANTARES's multi-catalog context and history, Fink's modular science modules, and Lasair's SQL filters and cross-matches.

This benchmark should therefore not reward recreating generic alert ingestion or another high-volume filtering broker. Instead evaluate the layer after and between brokers:

- Can it consume outputs from multiple existing brokers without claiming that broker agreement proves independent physical evidence?
- Can it preserve exact input packets, source lineage, receipt-time cutoffs, corrections and retractions across broker boundaries?
- Can it distinguish model probability, publisher assertion, verified source identity, calibrated physical measurement and scientific conclusion?
- Can it turn candidate hypotheses into discriminating follow-up experiments and measure realized decision value?
- Can an independent reviewer reproduce why the system accepted, rejected, deferred or abstained on an investigation?

Broker baselines should include the strongest available broker-specific filter/classifier, a union of broker candidate sets, a broker consensus policy, and an expert-selected subset. Match the input stream, source coverage, latency and follow-up budget. Broker agreement is not treated as statistically independent corroboration when the brokers share the same upstream alert packet, catalog, model family or cross-match service.

Source: [Rubin Observatory — Alerts and brokers](https://rubinobservatory.org/for-scientists/data-products/alerts-and-brokers) (updated 3 July 2026). This source documents the broker ecosystem and its normal capabilities; it is not a comparative performance study.

## 10. Cross-domain adapter change control

Adapter correctness includes endpoint lifecycle, not just conformance to a schema snapshot. The Copernicus Data Space Ecosystem's **16 October 2026** announcement says the new SFEOS/OpenSearch-backed STAC endpoint `https://stac.opensearch.dataspace.copernicus.eu/v1/` will run in parallel with the current `https://stac.dataspace.copernicus.eu/v1/` **until 12 January 2027**, when the old endpoint will be replaced by the new one. The announcement also describes faster response times, a fixed collections-search endpoint, more queryable attributes, thematic collection grouping and changed rate limits. The separate October 2026 catalogue API presentation describes OData as planned for decommissioning after 2028.

For satellite adapter evaluations:
- test both current and new SFEOS endpoint profiles explicitly before 12 January 2027;
- preserve endpoint identity, backend/profile version, query, pagination tokens, collection/item IDs and exact response digest;
- compare landing/conformance, collection and item search, queryables, pagination, sorting and asset hrefs;
- test redirects, schema/behavior drift, missing assets and rate-limit responses as separate failure modes;
- do not silently switch endpoint or assume catalogue equivalence;
- keep a dated endpoint-selection receipt in every replay so the same query can be reconstructed after the migration.

Sources: [Copernicus Data Space — New SFEOS STAC catalogue endpoint, 16 October 2026](https://dataspace.copernicus.eu/news/2026-10-16-new-sfeos-stac-catalogue-endpoint), [Catalogue APIs in CDSE (October 2026 PDF)](https://dataspace.copernicus.eu/sites/default/files/media/files/2026-10/1630_new_stac_catalogue_1.pdf), and [STAC API documentation](https://documentation.dataspace.copernicus.eu/APIs/STAC.html). This is endpoint lifecycle information, not evidence that all collections or assets are equivalent across endpoints.

## 11. Correlated-evidence and false-independence benchmark

A broker consensus is not a physical corroboration result. The benchmark must deliberately generate multiple records for one underlying observation and ensure that apparent multiplicity does not inflate confidence.

Compose the existing Mycelix dependency/corroboration owner ([#1243](https://github.com/Luminous-Dynamics/mycelix/issues/1243)) and verifier fault-domain/correlation owner ([#1371](https://github.com/Luminous-Dynamics/mycelix/issues/1371)). Do not create a second global dependency graph or an opaque scalar trust/independence score. The benchmark consumes a versioned, owner-defined dependency closure.

### Dependency dimensions to preserve

For each evidence item, preserve the dependency dimensions that matter to the claim and domain:

- **observation/artifact root** — the original measurement, alert, document or immutable payload;
- **upstream dataset/catalogue** — source records or databases from which the item was copied;
- **sensor/instrument/acquisition lineage** — device, deployment, site, sampling, channel and acquisition method;
- **gateway and transport** — broker, mirror, API, cache, relay or gateway;
- **operator/control domain** — organization or administrative controller;
- **trust/identity root** — signer, credential issuer, federation or key control;
- **clock/timebase and calibration/reference** — shared timing, calibration, power, environmental reference or common standard;
- **processing lineage** — shared feature extraction, reduction, classifier, mapping, translation or reconstruction implementation;
- **verifier lineage** — implementation, runtime, CI provider, package mirror and operator where verifier agreement is used.

The initial executable profile may be narrower, but omitted dimensions must be explicitly unknown—not implicitly distinct.

### Bounded relation outcomes

A consumer may classify the relation for a specified evidence purpose/profile as:

- `Correlated`: at least one material shared dependency is known;
- `Unknown`: no shared dependency is proven, but at least one required dimension is missing or unresolved;
- `IndependentForProfile`: all dimensions required by that frozen profile are present and distinct.

These labels do not prove mathematical or statistical independence. A profile-specific distinctness result is not physical truth, source authentication, authority or scientific correctness. The input dependency claims themselves require source qualification.

### Frozen adversarial families

1. Two astronomy brokers re-emit the same upstream VOEvent under different broker IDs, gateways and transport receipts.
2. Two national or commercial mirrors expose the same catalogue record through different APIs.
3. Two classifiers consume the same alert bytes but use different model names; shared observation root remains visible.
4. Two independent-looking sensors share a data-acquisition gateway or operator.
5. Two instruments share a clock, calibration reference, power source or site; profile-specific rules decide whether that is material.
6. Two independent source observations use the same processing implementation; preserve processing correlation without erasing the distinct observation roots.
7. AI summaries, embeddings or derived classifications of one observation never become new physical observations.
8. A dependency field is absent or a lineage lookup is unresolved; result remains `Unknown`.
9. A later-discovered common upstream source invalidates future independence claims without rewriting historical receipts.
10. Truly distinct observations with fully qualified required lineage provide a positive control.
11. Repeated delivery, rewrapping, re-signing and different file formats do not increment underlying observation-root count.
12. Reordering dependency records or input evidence cannot change the resulting disposition.

### Metrics and proposed hard gates

- False independence acceptance by dependency family: target **zero** on frozen adversarial controls.
- Unknown-lineage promotion rate: target **zero**.
- Known shared-root detection: **100%** on the frozen known-answer corpus.
- Correctly resolved unknown dependencies: report coverage and abstention; do not reward forced guesses.
- Pairwise evidence conflict detection and order invariance: **100%** on known-answer cases.
- Dependence-family recall, false independence rate, abstention/risk coverage, reviewer effort and additional lineage-resolution cost.
- Report the dimension-wise dependency vector, not only a count of sources or a confidence multiplier.

The critical SOTA comparison is not how many alerts or broker records are aggregated. It is whether the system detects when a high apparent corroboration count is actually a single shared source, while retaining genuinely distinct evidence and exposing unresolved common causes.

## 11A. Canonical receipt-time authority and as-known-at replay

The standalone lifecycle CLI accepts a caller-supplied receipt-time field for synthetic/retrospective replay and marks the result `unattested_manifest_assertion`. This is deliberate: hashing the source XML proves the bytes parsed, not when the fabric received them.

Production transaction-time queries must compose the canonical Mycelix DeSci receipt system rather than create a second receipt ledger:
- [mycelix-desci scientific event kernel](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-desci/src/core/src/scientific_events.rs) owns signed versioned event envelopes, canonical event hashes, stream IDs, sequences and append-only event history;
- [scientific authority audit](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-desci/src/core/src/scientific_authority_audit.rs) owns `ScientificAuthorityReceipt` / `SignedScientificAuthorityReceipt`, trusted service-key signature verification, event ID/hash/stream/sequence binding, receipt-time authority snapshots, and hash-linked receipt chains;
- [KNOW-ASOF-001 #5860](https://github.com/Luminous-Dynamics/mycelix/issues/5860) owns deterministic transaction-time/as-known-at read projections and their status distinctions (`receipt_attested`, `pending_reconciliation`, `legacy_unattested`, `unsafe_unattested`, `unknown`);
- [canonical statement/artifact identity #3022](https://github.com/Luminous-Dynamics/mycelix/issues/3022) owns exact source artifact digests, import receipts and provenance-family identity.

Keep the following clocks separate:
1. VOEvent source event time / phenomenon time;
2. Who/Date source creation time;
3. source publication/deposition time, where available;
4. source acquisition/import time, from the source adapter/import receipt;
5. canonical DeSci authority receipt time (server commit/known-at);
6. derived index/cache/model availability time, which requires its own materialization receipt.

A DeSci authority receipt proves that a canonical event and authorization decision were committed by the service at its receipt time under the configured trust model. It does not prove the source observation is true, when the source first existed, or when a downstream index/model incorporated it. A packet with `pending_reconciliation`, `legacy_unattested`, `unsafe_unattested` or `unknown` receipt status must not silently enter a qualified as-known-at frontier.

Required negative controls:
- signer backdates `occurred_at` before the receipt; a cutoff earlier than the server receipt must exclude it;
- caller submits a forged or backdated `received_at`; governed append ignores caller time and stamps server receipt time;
- service receipt signature, trusted key, authority snapshot, event hash, stream, sequence or previous receipt hash is altered;
- pending receipt has not committed, authority reconciliation is incomplete, or a receipt-chain fork is present;
- legacy pre-cutover event has no receipt and is returned only as `legacy_unattested`;
- later correction/retraction does not change an earlier K snapshot;
- source acquisition is later than publication, and neither timestamp is substituted for canonical commit time;
- an embedding/index built after K is not described as available at K.

The current standalone package does not verify signed DeSci receipts. It is a packet parser and event-graph replay component with an explicit unattested cutoff basis. The canonical DeSci adapter/API is a separate integration step owned by #5860; do not implement cryptographic verification or a second trust registry here.

## 12. Release rule

A public benchmark release must include the versioned raw-source corpus manifest, parser version, ingress receipt evidence, per-case rights and provenance, frozen split and cutoff definitions, expected outputs, evaluator source, baseline configurations, raw run receipts and a limitations report. Keep the hidden evaluation set private until a run is submitted; then publish enough evidence to reproduce the result without leaking future test labels.
