# Scientific Observability Fabric — Research Architecture v0.1

**Programme:** [SCI-OBS-001](https://github.com/Luminous-Dynamics/mycelix/issues/6141)  
**Status:** Research design / proposed contract. Not implemented or physically qualified.  
**Scope:** Ocean, Earth-observation satellite, space situational awareness, astronomy and terrestrial observatories.  
**Architecture owners:** Mycelix (evidence/provenance/rights), Symthaea (scientific inference/investigation), Symtropy (physical models/qualification), domain applications including Sol-Atlas (specialized evidence and user workflows).

## 1. Thesis

Build a federated scientific observation and investigation fabric, not a universal sensor dashboard and not a blockchain for raw telemetry.

The fabric should make it possible for independently operated observatories to contribute, discover, qualify, cross-reference and investigate observations while retaining control of source data, interpretation, access and physical instruments.

The target loop is:

```
source observations
      ↓
source-specific admission and evidence qualification
      ↓
time / space / identity / calibration / rights reconciliation
      ↓
anomaly candidates and cross-source association
      ↓
competing hypotheses with testable predictions
      ↓
value-of-information investigation proposal
      ↓
human / operator authorization
      ↓
follow-up observations
      ↓
independent replay and prediction scoring
      ↓
reproducible claim, correction or retraction
```

The system should be able to say **“we do not know”** precisely. Missing data, uncalibrated instruments, unknown provenance, out-of-domain inference and a qualified non-detection are different states.

## 2. Existing substrate: what we can reuse

### 2.1 Active Mycelix DeSci

The active `mycelix-desci/` subtree in the Mycelix repository is the intended scientific evidence/provenance kernel. Its current canonical path already includes:

- research objects and atomic claims;
- content-addressed evidence-artifact references;
- signed, versioned scientific events using an explicit canonical codec;
- per-stream sequence, previous hash, optimistic concurrency, idempotency and hash chaining;
- append receipts and deterministic, disposable evidence projections;
- typed attestations and correctable / withdrawable evidence;
- credential registry, actor/key/organization/role resolution and governed authority;
- explainable, versioned evidence assessments.

The existing `EvidenceArtifact` is intentionally compact: artifact ID, content hash, media type, locator, optional licence and availability. It is a useful anchor, but it is **not by itself** a complete observatory record. Observation-specific metadata, calibration and time/frame semantics belong in a versioned profile that references the artifact and preserves its source identity.

The older mutable claim, trust-score and E0–E4 surfaces are compatibility interfaces. They must not be used as the canonical observation record or as a proxy for physical truth. A signature proves that a key signed bytes under a protocol; it does not prove that a sensor measured a real phenomenon.

### 2.2 Mycelix Space

`mycelix-workspace/mycelix-space/` is an existing specialist space-domain client. Its observation model includes optical, radar, RF and laser measurements, sensor identity, observation time, submission time, ground location, measurement-specific uncertainty, orbital state and covariance-aware fusion.

Reuse that vertical for orbital observations and space situational awareness. Do not retrofit it into a universal ocean / satellite-imagery / astronomy schema. Its current observation shape does not by itself express every generic observatory requirement (for example, a complete source-artifact manifest, purpose-bound data rights, calibration-validity profile and clock/time-scale uncertainty).

### 2.3 Sol-Atlas

Sol-Atlas is a specialist terrestrial/Earth-observability client. Its current research direction already includes:

- source and coordinate conformance;
- target- and environment-conditional sensor observability;
- explicit non-detection states;
- competing utility-network hypotheses;
- project-grouped transfer and uncertainty;
- evidence-dependent revalidation;
- decision-loss-based investigation tournaments;
- worst-project reporting and rights constraints.

Compose these capabilities through an adapter and shared receipts; do not move the entire Sol-Atlas domain model into Mycelix.

Related programmes:
- Sol-Atlas [#761 — multimodal Earth-observability benchmark](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/761)
- Sol-Atlas [#771 — decision-grade observability and investigation](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/771)
- Sol-Atlas [#779 — joint utility-network hypotheses](https://github.com/Luminous-Dynamics/sol-atlas-leptos/pull/779)
- Sol-Atlas [#793 — investigation tournament and campaign aggregation](https://github.com/Luminous-Dynamics/sol-atlas-leptos/pull/793)
- Sol-Atlas [#359 — federated digital-twin interoperability](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/359)

### 2.4 Existing assurance work to compose

Do not create a parallel evidence store, authority engine, dependency graph or generic trust score. Integrate with:

- Mycelix [#6140 — observation provenance and evidence coverage](https://github.com/Luminous-Dynamics/mycelix/issues/6140)
- Mycelix [#1243 — evidence-dependency and corroboration-independence](https://github.com/Luminous-Dynamics/mycelix/issues/1243)
- Mycelix [#1371 — verifier independence and correlated-failure accounting](https://github.com/Luminous-Dynamics/mycelix/issues/1371)
- Mycelix [#4835 — independent evaluation and oracle separation](https://github.com/Luminous-Dynamics/mycelix/issues/4835)
- Mycelix [#6069 — bitemporal reconstruction and contestability](https://github.com/Luminous-Dynamics/mycelix/issues/6069)
- Mycelix [#5395 — standards artifact conformance and semantic-loss testing](https://github.com/Luminous-Dynamics/mycelix/issues/5395)

## 3. Core architectural decisions

### ADR-01 — Federate evidence; do not centralize all raw data

**Decision:** Raw telemetry, imagery, radargrams, spectra and high-rate streams remain with the source owner or a rights-approved specialist object store by default.

Mycelix should hold or reference:
- signed observation and source manifests;
- content digests and byte-level identity;
- source-native identifiers and schema versions;
- provenance, custody, attestations and correction history;
- calibration references and their qualification state;
- rights, purpose and disclosure decisions;
- analysis-run and reproducibility receipts;
- links to hypotheses, claims and follow-up observations.

Replicate large payloads only when the source owner permits it and the storage, cost, security and retention model justifies it. A content hash does not grant the right to retrieve, redistribute or republish the payload.

### ADR-02 — Separate the evidence plane from domain interpretation

**Decision:** A shared evidence plane supplies identity, provenance, rights, temporal semantics, integrity and reproducibility. Domain applications own scientific interpretation and physical meaning.

No generic schema should imply that a radargram, ocean CTD profile, satellite raster, photon-counting event and orbital range measurement are interchangeable.

### ADR-03 — Preserve distinct epistemic states

**Decision:** Never collapse authenticity, integrity, source independence, calibration, quality, empirical support, statistical calibration, reproducibility, authority and rights into one score.

Examples:
- A valid signature does not establish sensor calibration.
- A current calibration does not establish that a model is valid for the target domain.
- Two mirrors of one file are one source lineage, not two observations.
- An independent observer may still have incomplete temporal or spatial coverage.
- A high-trust participant cannot turn an unobserved event into an observed one.
- An anomaly is not a discovery.

Use explicit states such as `Qualified`, `PartiallyQualified`, `Unqualified`, `OutOfDomain`, `Unknown`, `Conflicted`, `Withdrawn`, `Retracted`, and `NotRun` as appropriate to each contract. Do not treat this illustrative vocabulary as a final enum until owner-native state semantics are reconciled.

### ADR-04 — Domain-specific time and space profiles

**Decision:** Do not coerce every observation into UTC + latitude/longitude.

Preserve native time scales, coordinate frames, epochs, vertical datums, units, uncertainty and sampling support. A conversion is a versioned evidence-producing operation with input and output identity, method, parameters, software and uncertainty propagation. An unavailable transform remains unavailable.

### ADR-05 — Autonomy proposes; owners authorize

**Decision:** Symthaea may analyze authorized archives, generate hypotheses, rank follow-up observations and issue investigation proposals. It may not directly command a telescope, satellite, underwater vehicle, radar or other instrument under this programme.

Tasking requires an explicit capability and approval from the instrument owner/operator, plus any safety, regulatory, ecological, cultural or site-specific review. A proposal, authorization, execution attempt and observed outcome are separate records.

## 4. Observation envelope and domain profiles

### 4.1 Common observation envelope

The common envelope should be deliberately small, versioned and additive. It should reference native source records rather than replace them.

| Field group | Required semantics |
|---|---|
| Identity | Stable fabric event ID, source-native ID, source system, schema/profile version |
| Assertion | Issuer/agent, signed assertion reference, source/organization identity |
| Observed property | Domain concept URI or source-native code, quantity kind and method |
| Time | Phenomenon time or interval, source result time, received/ingested time, knowledge time, time scale, clock source and uncertainty |
| Space | Domain reference system, frame/epoch, support geometry and uncertainty where meaningful |
| Instrument | Platform, instrument/sensor, detector/channel, deployment, configuration, firmware/software revision |
| Calibration | Calibration ID, artifact reference, validity interval, traceability/qualification state and applicability domain |
| Result | Typed value or immutable artifact reference; units, dimensions, encoding, byte length and digest |
| Quality | Source QC flags, missingness, saturation, detection limit, acquisition conditions and exclusions |
| Processing | Parent inputs, pipeline/model/version/configuration, transforms, parameters and reproducibility receipt |
| Rights | Owner, licence, permitted purpose, disclosure and retention constraints, jurisdiction/classification |
| Relations | Derived-from, corrected-by, supersedes, conflicts-with, retracts and corroborates-under-profile |

Do not silently fill absent fields with plausible defaults. Every field has a typed state: present and validated; present but unqualified; absent; not applicable under this profile; unsupported by this adapter; or unknown.

### 4.2 Domain profiles

| Domain | Preserve native semantics | Candidate interoperability |
|---|---|---|
| Terrestrial | CRS, vertical datum, coordinate epoch, survey method, support, depth reference, instrument and calibration | OGC API Features, SensorThings, STAC for imagery, AGS/SEG-Y and domain profiles in Sol-Atlas |
| Ocean | Platform, water-column/depth/pressure coordinate, vertical datum, timebase, QC flags, sampling interval, method and deployment | OGC SensorThings/STAplus, ERDDAP, OOI and IOOS-compatible services |
| Earth-observation satellite | STAC Item/Collection/Asset, platform/instrument, acquisition interval, geometry, processing baseline, bands, calibration and quality metadata | STAC 1.1 / STAC API, Copernicus Data Space |
| Space situational awareness | Orbital frame and epoch, time scale, observer geometry, sensor site, measurement type, covariance and object association | Existing Mycelix Space model, source-native TLE/CDM formats and relevant tracking interfaces |
| Astronomy | Celestial reference frame, equinox/epoch, observation time scale, alert identity, photometry/spectra, instrument/filter and broker/catalogue lineage | IVOA archive/query standards, VOEvent and community alert brokers |

The first adapters should be read-only. Standard conformance is measured separately from physical truth or instrument qualification.

### 4.3 Illustrative envelope (not a schema)

```json
{
  "profile": "mycelix-observation-core/0.1",
  "observation_id": "urn:uuid:...",
  "source": {
    "system": "source-native-id",
    "record_id": "native-record-id",
    "schema": "source-schema/version",
    "issuer": "did-or-source-identity",
    "assertion_ref": "signed-source-attestation"
  },
  "phenomenon": {
    "concept": "domain-concept-uri",
    "observed_at": {
      "value": "source-native-time",
      "time_scale": "profile-defined",
      "uncertainty": "explicit-or-unknown"
    }
  },
  "instrument": {
    "platform": "source-platform-id",
    "instrument": "source-instrument-id",
    "configuration": "immutable-config-reference",
    "calibration": "calibration-artifact-and-validity-reference"
  },
  "result": {
    "media_type": "application/octet-stream",
    "byte_length": 0,
    "digest": {
      "algorithm": "sha256",
      "value": "..."
    },
    "locator": "rights-qualified-source-uri"
  },
  "rights": {
    "owner": "source-owner",
    "purpose": ["research-purpose"],
    "disposition": "explicit-profile-state"
  },
  "lineage": {
    "derived_from": [],
    "supersedes": [],
    "correction_of": []
  }
}
```

This sketch intentionally leaves domain-specific coordinate and measurement payloads in profile-defined attachments. It is not yet a normative Rust type, JSON Schema or API contract.

## 5. Federated topology

A first implementation should have three separable planes:

1. **Source plane:** instrument operators, mission systems, archives, data centres, brokers, APIs and object stores retain their native records and policy.
2. **Evidence plane:** Mycelix DeSci records signed manifests, events, custody/authority history, rights decisions, derivation links and reproducibility receipts.
3. **Analysis plane:** domain applications and Symthaea query permitted references, acquire authorized data, run versioned analysis and publish derived evidence.

A federation peer may:
- publish a minimal source/profile/capability description;
- expose a catalogue or query endpoint subject to source policy;
- return signed manifests and scoped artifact references;
- retain raw payloads locally;
- issue a new immutable event when an observation is corrected or withdrawn;
- synchronize allowed metadata and receipts while offline/partitioned;
- record which evidence was unavailable at a given decision cutoff.

A peer must not be able to:
- rewrite another source's observation;
- grant rights on behalf of another owner;
- convert a cached copy into a new independent source;
- imply that a missing record means no event occurred;
- make a local trust judgement globally authoritative;
- automatically publish precise critical-infrastructure, sensitive ecological or restricted astronomical data.

Holochain/DHT mechanisms may be useful for peer-distributed indexes and agent-authored records where the selected profile benefits from them. They are not a requirement to push every sample, pixel, photon event or telemetry packet through a DHT. High-rate transport and bulk storage should use specialist systems and be connected through explicit receipts and adapters.

## 6. Symthaea: anomaly detection to scientific investigation

### 6.1 Do not begin with a universal anomaly model

A single model across orbital mechanics, ocean chemistry, satellite imagery, astronomical transients and GPR is not a scientifically defensible P0.

Start with domain-specific detectors and robust baselines:
- range/physics residuals and covariance consistency for orbital observations;
- sensor-specific QC, drift, change-point and spectral/physical residuals for ocean observations;
- scene-level change, cloud/quality masking and cross-date consistency for Earth imagery;
- broker/classifier baselines, temporal cross-match and known-variable filtering for astronomical alerts;
- target/environment-conditional likelihoods for subsurface observations.

Only introduce learned models where the labels, reference truth, acquisition process and held-out evaluation are understood.

### 6.2 Candidate classification

The system should explicitly distinguish:
- candidate physical phenomenon;
- known/expected event;
- sensor fault or calibration drift;
- clock, timebase or georeferencing fault;
- acquisition or transmission defect;
- processing or model artifact;
- duplicate/correlated report;
- out-of-domain observation;
- unknown / unresolved.

Multiple causes may remain plausible. The system should preserve that ambiguity rather than force one label.

### 6.3 Competing hypotheses

For an anomaly candidate, Symthaea should record:
- hypothesis identity and version;
- evidence references and dependency lineage;
- prior and posterior probabilities or another declared uncertainty representation;
- predicted observable consequences;
- observations that would support, contradict or distinguish each hypothesis;
- known confounders and model applicability;
- stopping criteria and expected information/decision value.

A hypothesis must make predictions that can be scored later. Model-generated prose without testable consequences is not a discovery artifact.

### 6.4 Investigation selection

Candidate follow-up actions should be evaluated under a common feasible action set and budget.

Baselines:
1. expert-selected follow-up;
2. fixed schedule / fixed-grid observation;
3. random eligible observation;
4. cost-only selection;
5. anomaly-score-greedy;
6. entropy-greedy;
7. expected value of information / expected net benefit of sampling;
8. Symthaea policy;
9. hidden-truth oracle, used only as a non-deployable upper bound in synthetic evaluation.

The optimizer may rank only actions that pass owner, rights, safety, resource, timing and instrument-availability constraints. Its reported information gain must remain separate from realized scientific value.

### 6.5 Independent replay

Before a prospective follow-up:
- freeze the hypothesis set, predictions, action set, budgets, scoring method and stopping rules;
- bind exact model and data digests;
- protect blind outcomes from the proposing system;
- ensure the evaluator cannot be changed by the candidate under evaluation.

After follow-up:
- append observations and corrections;
- score the predeclared predictions;
- report misses, false alarms, conflicting evidence and indeterminate outcomes;
- preserve the failed hypothesis and its lineage.

A discovery claim requires domain-scientist review and evidence appropriate to the claim. Symthaea cannot self-certify its own finding.

## 7. Symtropy: physical models and discrepancy

Symtropy should own qualified forward models and physical assumptions. Separate these questions:

1. analytical reference correctness;
2. numerical convergence and discretization error;
3. implementation/backend agreement;
4. sensor and acquisition-model validity;
5. agreement with controlled physical experiments;
6. transfer to field conditions and independent projects.

Agreement between two solvers is not proof of physical truth when both share the same assumptions, input data, approximations or implementation lineage. Record common-cause dependencies.

Anomaly residuals should carry the exact forward-model version, calibration state, input evidence and applicability domain. Model discrepancy must be visible rather than absorbed into a falsely precise posterior.

## 8. Benchmark programme

### 8.1 Evidence and adapter conformance

Primary metrics:
- source-native identity preservation;
- exact byte-digest correctness;
- field/semantic preservation and explicitly accounted loss;
- coordinate/time/unit conversion known-answer correctness;
- calibration/currentness and applicability handling;
- correction/retraction and bitemporal replay correctness;
- rights and purpose enforcement;
- replay determinism and idempotency.

Initial conformance targets:
- 100% pass on frozen positive contract vectors;
- zero silent field drops, identity merges, guessed units or time/frame substitutions;
- zero unauthorized egress in the adversarial corpus;
- every unsupported capability surfaced as partial/unsupported, not complete;
- every transformed claim bound to exact inputs, transform implementation/version and output digest.

### 8.2 Anomaly and event detection

Do not report AUROC alone. For each domain, report:
- event-level precision/recall and recall at a predeclared false-alert ceiling;
- false alerts per relevant exposure unit (instrument-hour, observation, track-km², scene-area or sky-area);
- event localization/association error where ground truth supports it;
- time-to-detection and alert latency;
- duplicate and missed-event rates;
- Brier score, log score and reliability diagrams for probabilistic outputs;
- selective risk versus coverage and abstention;
- OOD detection and performance degradation under domain shift;
- human review burden per accepted event.

The denominator and unit must be declared for each domain. Do not compare one domain's false alerts/hour directly with another's false alerts/deg².

### 8.3 Hypothesis quality

Measure:
- proper scoring of predeclared outcomes;
- calibration of hypothesis probabilities;
- competing-hypothesis discrimination;
- independent evidence update quality;
- false-discovery and confirmation-bias controls;
- robustness to contradictory, delayed and corrected evidence;
- exact replay of analysis and result receipts;
- independent replication rate.

### 8.4 Investigation policy

Compare expert, fixed-schedule, random, cost-only, anomaly-greedy, entropy-greedy, EVSI/ENBS and Symthaea under identical:
- action catalogue and eligibility rules;
- data availability and information cutoff;
- cost, time and resource budgets;
- instrument capabilities and downtime;
- rights and operational constraints;
- loss function / scientific utility;
- scoring and holdout.

Report mean and worst-domain/worst-project outcomes, realized loss/regret, cost, latency, operator/reviewer time, missed opportunities and safety/rights violations. Never let a single pooled average conceal a domain failure.

A stretch target of 20% lower out-of-sample decision/scientific loss at equal resource budget (or equivalent quality at 20% lower cost) is a hypothesis, not a current result. Freeze the effect threshold only after baseline variance, sample size and power are established.

### 8.5 Federation and resilience

Minimum adversarial corpus:
- duplicate / replayed source data;
- mirrors incorrectly counted as independent;
- shared upstream catalogue or broker;
- unknown common operator or trust-root dependency;
- compromised / rotated source key;
- stale or expired calibration;
- late and backdated correction;
- out-of-order timestamps and clock skew;
- partial source outage and incomplete pagination;
- network partition and delayed reconciliation;
- unauthorized purpose / attempted egress;
- source withdrawal and inaccessible artifact;
- candidate model attempts to read blind truth or modify evaluator identity.

Measure ingestion completeness, idempotency, p50/p95/p99 latency, bandwidth, storage, memory, recovery time and operator burden. Report hardware and source constraints; do not publish synthetic throughput as production capability.

## 9. Industry landscape and differentiation

Build on standards and services rather than competing with them:

- **OGC SensorThings / STAplus:** interoperable sensor observations and ownership-aware sharing.
- **STAC / STAC API:** discovery and representation of Earth-observation assets.
- **IVOA / VOEvent:** astronomical archive interoperability and transient-event packets.
- **IOOS / OOI / ERDDAP:** ocean observing, data discovery and access.
- **Rubin community alert brokers:** high-rate alert filtering, crossmatching, classification and follow-up prioritization.
- **Sol-Atlas:** terrestrial spatial evidence, subsurface reconstruction and decision-grade investigation.
- **Mycelix Space:** orbital-object observations, covariance-aware fusion and conjunction workflows.

The competitive edge should be:
1. source-native meaning preserved across federated boundaries;
2. evidence and rights that are portable and independently auditable;
3. explicit dependency-aware corroboration;
4. hypotheses with testable predicted consequences;
5. decision-aware investigation across independent projects;
6. durable negative, failed and superseded results;
7. human/operator sovereignty over physical tasking.

Do not claim superiority over an alert broker on raw alert throughput, over an ocean data portal on archive coverage, or over a geospatial platform on cartographic maturity. Compare the common evidence/investigation capabilities on matched tasks.

## 10. Phased delivery

| Phase | Deliverable | Acceptance boundary |
|---|---|---|
| P0 | Observation core + domain profile RFC, threat model, owner map | Contract reviewed; no implementation claim |
| P1 | Strict conformance corpus, validators and receipts | Frozen known-answer and negative controls pass |
| P2 | Read-only adapters for bounded astronomy, ocean and satellite sources | Rights-qualified sample manifests, exact hashes and mapping-loss reports |
| P3 | Domain-specific anomaly challenge suites | Matched classical baselines and blind event/fault scoring |
| P4 | Symthaea hypothesis and investigation replay | Predeclared prediction scoring and equal-budget comparator |
| P5 | Two independently administered federation peers | Rights-aware exchange, offline replay and common-cause accounting |
| P6 | Prospective human-authorized scientific pilot | Independent domain review and separately qualified physical procedures |

Recommended first data choices:
- Astronomy: a bounded public Rubin broker/alert replay or public VOEvent-compatible dataset; do not try to replace a production broker or begin with the full live stream.
- Ocean: one documented OOI or ERDDAP time series with QC flags, units, timebase and source metadata.
- Earth imagery: a small Copernicus Data Space STAC query and one rights-cleared asset manifest.
- Space: existing Mycelix Space fixtures and source-native orbital observation profiles.
- Terrestrial: Sol-Atlas synthetic contracts first; UT-Gold-D only after the dataset admission, rights, truth-tier and blind-split gates in Sol-Atlas #783.

A first pilot should be retrospective/read-only. No external instrument control is needed to demonstrate value.

## 11. Threat model and safety boundaries

Threats include forged or replayed source assertions, compromised instrument keys, sensor spoofing, calibration drift, clock manipulation, geolocation corruption, duplicate/mirrored evidence, colluding reviewers, poisoned labels, data rights abuse, source outage, malicious model output, evaluator leakage, confirmation bias and unsafe tasking.

Required controls:
- verify signatures and byte digests separately;
- bind keys to governed source identities and validity intervals;
- preserve key compromise and revocation history;
- account for common-source, common-operator and common-software dependencies;
- qualify calibration for target and environment;
- use append-only correction, withdrawal and supersession;
- enforce purpose-bound access and egress controls;
- isolate model execution and freeze evaluation holdouts;
- keep external actions human/operator-authorized;
- represent unknown outcomes explicitly.

No consensus quorum, confidence score, anomaly alert or model-generated hypothesis can override rights, safety or instrument-owner authority.

## 12. Open questions for the implementation phase

1. Which source/profile owner supplies authoritative instrument identity and calibration status for each pilot?
2. Which adapters can legally retain source payload bytes, and which may retain only signed metadata and digests?
3. What are the smallest cross-domain fields that can be mapped losslessly without erasing domain-specific uncertainty?
4. Which Mycelix components should host catalogues versus only signed source references and authority?
5. How do we prove adapter independence from source, mapping implementation and common trust roots?
6. Which pilot question has sufficient known events, negative examples and independent follow-up evidence to support a blind evaluation?
7. What is the best first human-in-the-loop operating workflow for Symthaea without any direct instrument control?

## 13. Claim ceiling

This document is a research design. It does not establish:
- an implemented universal observation schema or production data fabric;
- live feeds, telemetry rights or permission to redistribute external data;
- calibration or sensor qualification for any domain;
- scientific discovery performance by Symthaea;
- physical-model validation by Symtropy;
- independent replication of a finding;
- authority to task, steer or control any instrument.

A successful synthetic contract test is software evidence, not physical evidence. A signed source event is evidence of an assertion, not proof that the physical event occurred. A promising anomaly is a candidate for investigation, not a discovery.
