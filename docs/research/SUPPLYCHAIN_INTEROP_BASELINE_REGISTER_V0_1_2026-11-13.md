# Supply-Chain Interoperability Baseline Register V0.1 — 2026-11-13

**Status:** source-qualified candidate register; no server was deployed or benchmarked.  
**Parent:** [SUPPLYCHAIN-INTEROP-001 #6105](https://github.com/Luminous-Dynamics/mycelix/issues/6105)  
**Normative GS1 owner:** [MYC-GS1-CONFORMANCE-001 #5395](https://github.com/Luminous-Dynamics/mycelix/issues/5395)  
**Related mapping:** [SUPPLYCHAIN_INTEROP_MAPPING_V0_1.json](SUPPLYCHAIN_INTEROP_MAPPING_V0_1.json)

## 1. Why this register exists

An interoperability benchmark needs two different things that must not be conflated:

1. **Normative conformance instruments:** the exact GS1 standard artefacts and their declared validators, pinned by content digest.
2. **Independent implementation baselines:** real EPCIS repositories/servers that can capture, query, export and round-trip the same frozen event corpus.

The official GS1 JSON Schema and SHACL artefacts are authoritative validation inputs, not a server implementation. A server's own self-description or its own profile validator cannot independently certify itself. This register identifies candidate implementation families and their boundaries so #6105 can reuse the conformance and empirical baseline lane already owned by #5395.

## 2. Normative artifact versioning — preserve the specification's actual references

The current GS1 EPCIS artefacts index lists EPCIS/CBV 2.0.1 and exposes a 2.0.1 artefact set:
- [EPCIS/CBV artefacts index](https://ref.gs1.org/standards/epcis/artefacts)
- [EPCIS 2.0.1 artefacts page](https://ref.gs1.org/standards/epcis/2.0.1/artefacts)

However, the published EPCIS 2.0 specification text served from the 2.0.1 standard page identifies its own normative JSON/JSON-LD validation dependencies using the versioned **2.0.0** URLs. In particular, section 10.3 references:
- [EPCIS 2.0.0 JSON Schema](https://ref.gs1.org/standards/epcis/2.0.0/epcis-json-schema.json)
- [EPCIS 2.0.0 JSON-LD context](https://ref.gs1.org/standards/epcis/2.0.0/epcis-context.jsonld)
- [EPCIS 2.0.0 SHACL shapes](https://ref.gs1.org/standards/epcis/2.0.0/epcis-shacl.ttl)
- [CBV 2.0.0 ontology JSON-LD](https://ref.gs1.org/standards/epcis/2.0.0/cbv-ontology.jsonld)

The exact 2.0.0 JSON Schema is currently retrievable and declares JSON Schema Draft 07. The browser reader cannot parse the Turtle and JSON-LD media types for the other direct artefacts; this is a reader/media-type limitation, not evidence that the resources return 404. Prior tool-observed 404 reports in #5395 remain historical observations, not a confirmed global outage.

**Rule:** #5395 must preserve two separate source sets:
- the exact 2.0.0 artefacts referenced by the normative conformance clauses; and
- the currently indexed 2.0.1 artefacts, captured separately for version comparison and change analysis.

Do not silently substitute a 2.0.1 artefact for a 2.0.0 URL named by the normative text. Resolve, capture and hash the exact bytes, final URLs and media types with an independent HTTP client. Run the JSON Schema under its declared Draft 07, JSON-LD expansion against the exact context, and SHACL against the selected shapes as separate validation stages and receipts. The custom Mycelix cross-invariant verifier remains a distinct fourth stage.

The 2026-11-13 page/PDF/schema comparison is recorded in the [GS1 artifact-version differential audit](GS1_EPCIS_ARTIFACT_VERSION_DIFFERENTIAL_AUDIT_2026-11-13.md) and its [machine-readable record](GS1_EPCIS_ARTIFACT_VERSION_DIFFERENTIAL_AUDIT_2026-11-13.json). In particular, the current index exposes a 2.0.1 JSON Schema (its declared `$id` names 2.0.1), while the linked EPCIS PDF identifies itself internally as Release 2.0 / June 2022 and section 10.3 names the 2.0.0 JSON Schema and SHACL URLs. Both source sets remain separate; no raw artifact hashes or validator outputs are claimed.

## 3. Implementation candidates

### A. FasTnT EPCIS — first independent server candidate

- Repository: [louisaxel-ambroise/epcis](https://github.com/louisaxel-ambroise/epcis)
- Candidate revision: `main` at `1df0e51f640b43a63da8080d4ca85365b47d328f` (2026-11-11 16:15 UTC)
- License: Apache-2.0 (repository `LICENSE`)
- Declared stack: .NET 10, Entity Framework Core; SQLite, PostgreSQL and SQL Server providers.
- Project README describes EPCIS 1.2 and 2.0 support, JSON/XML capture/query, named queries, discovery endpoints, subscriptions and callbacks.
- Source inspection confirms:
  - `CaptureEndpoints.cs`: `POST /capture`, `POST /events`, capture detail/list routes.
  - `EventsEndpoints.cs`: `GET /events`, event-by-ID and business-step/location/read-point/disposition query routes.
  - `QueriesEndpoints.cs`: named-query list/create/get/delete and `GET /queries/{queryName}/events`.
  - `DelegateFactory.cs`: response headers advertise `GS1-EPCIS-Version: 2.0`, `GS1-EPCIS-Min: 1.2`, `GS1-EPCIS-Max: 2.0`; source also documents restrictions on capture error behaviour and EPC format.
- The source does **not** establish EPCIS 2.0.1 support, full normative conformance, correct content negotiation across every endpoint, complete extension semantics, or semantic round-trip preservation. These require black-box probes and the official #5395 validators.

**Disposition:** preferred first *candidate* for an independent, bounded EPCIS 2.0 compatibility profile. Do not label it a conformant EPCIS 2.0.1 baseline until its exact protocol profile, headers, query behaviour, failure semantics and official artifact checks pass. Its declared maximum version is 2.0, so a strict 2.0.1 comparison must keep that capability gap visible.

Exact inspected sources:
- [README at candidate revision](https://github.com/louisaxel-ambroise/epcis/blob/1df0e51f640b43a63da8080d4ca85365b47d328f/README.md)
- [Capture endpoints](https://github.com/louisaxel-ambroise/epcis/blob/1df0e51f640b43a63da8080d4ca85365b47d328f/src/FasTnT.Host/Endpoints/CaptureEndpoints.cs)
- [Event query endpoints](https://github.com/louisaxel-ambroise/epcis/blob/1df0e51f640b43a63da8080d4ca85365b47d328f/src/FasTnT.Host/Endpoints/EventsEndpoints.cs)
- [Named query endpoints](https://github.com/louisaxel-ambroise/epcis/blob/1df0e51f640b43a63da8080d4ca85365b47d328f/src/FasTnT.Host/Endpoints/QueriesEndpoints.cs)
- [Version and protocol headers](https://github.com/louisaxel-ambroise/epcis/blob/1df0e51f640b43a63da8080d4ca85365b47d328f/src/FasTnT.Host/Endpoints/Utils/DelegateFactory.cs)

### B. OpenEPCIS Community Edition — second implementation family

- Repository: [openepcis/epcis-repository-ce](https://github.com/openepcis/epcis-repository-ce)
- Candidate revision: `main` at `e81a857771c4c4a2d12e62b60a2dbc094551aa6e` (2026-01-10 14:45 UTC)
- License: Apache-2.0 (repository `LICENSE`)
- README describes an EPCIS 2.0 repository with JSON-LD and REST capture/query, container deployment, Quarkus, Kafka and OpenSearch.
- The README's walkthrough identifies a capture endpoint and event query path. Exact server behaviour and profile support still need black-box qualification.

**Disposition:** useful second server implementation candidate for cross-vendor semantic exchange. Its repository's latest commit is older than the FasTnT candidate's and its full-stack dependencies are materially different; record deployment effort and versions as part of the benchmark. Do not infer conformance from project claims.

OpenEPCIS also publishes Event Sentry, a profile/schema validator, and a document-validation service. These are in the same OpenEPCIS/benelog product family as the repository, so they are **not independent adjudicators** of OpenEPCIS Community Edition. They may be recorded as same-family diagnostic tools, not as a second independent implementation or gold judge.

Exact sources:
- [OpenEPCIS Community Edition](https://github.com/openepcis/epcis-repository-ce)
- [Event Sentry](https://github.com/openepcis/openepcis-event-sentry)
- [Document validation service](https://github.com/openepcis/openepcis-document-validation-service)

### C. GS1 EPCIS Sandbox — official fixture and diagnostic tool, not a server

- [GS1 EPCIS Sandbox](https://epcis-sandbox.gs1.org/) is described by GS1 as a web tool for event generation/viewing, XML/JSON-LD validation and format conversion.
- Use it to create and inspect known-answer fixtures and to identify user-visible conversion differences.
- It is not an EPCIS repository/server, not an independent implementation, and not a substitute for pinned normative JSON Schema, JSON-LD and SHACL validation receipts.

The older [FRE E PCIS service](https://freepcis.gs1.org/) is not a current EPCIS 2.0 comparator. Its current FAQ states that it is a development/test repository limited to 25 events and supports EPCIS 1.2; GS1's architecture material characterizes it as a legacy, no-longer-maintained tool superseded by EPCIS Sandbox. Retain it only as an optional EPCIS 1.2 backward-compatibility probe. Do not include it in EPCIS 2.0/2.0.1 performance comparisons.

### D. NIST IR 8536 reference implementation — research reference, access unresolved

NIST IR 8536 is a manufacturing traceability meta-framework, not a product certification or one mandatory API. Its final September 2026 Appendix F describes a minimum viable reference implementation jointly developed by NIST and MITRE. It is explicitly an **early-stage research artifact for laboratory experimentation, prototyping and exploration, not production-grade software**.

The final report says the illustrative MVP:
- models three independent ecosystems: a microelectronics manufacturer (ME-001), an operational-technology integrator (OT-001), and a critical-infrastructure acquirer (CI-001);
- defaults to simple text files as trusted repositories, with NoSQL available as an option;
- includes a lightweight Python web dashboard to view ecosystem records and navigate traceability links;
- demonstrates Make / Ship / Receive / Assemble / Employ event sequences.

Appendix F.4 links the source repository: https://github.com/usnistgov/ir8536-reference-implementation. However, on 2026-11-13 the connected GitHub API returned 404 for the repository, and GitHub repository search returned no result. NIST's 2026-09-09 announcement had said the repository would be made publicly available following the approval process. This is an **unresolved publication/access discrepancy**, not proof that the source does not exist or that NIST's link is erroneous.

**Disposition:** keep the NIST RI as a conceptual and lab-reference target only. Do not make it a runnable benchmark arm until the repository is retrievable and its exact commit/tag, source bytes, license, dependencies, tests and maintenance state are inspected. It is not a substitute for an independently implemented GS1 EPCIS repository/server.

Sources:
- Final IR: https://csrc.nist.gov/pubs/ir/8536/final
- NIST announcement: https://www.nist.gov/news-events/news/2026/09/finalized-manufacturing-supply-chain-traceability-meta-framework
- IR PDF Appendix F: https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=962361

## 4. Qualification sequence

1. **Source intake:** capture exact normative artifacts and digest them; preserve both 2.0.0 normative URLs and the separate 2.0.1 index.
2. **Capability probe:** start each candidate in an isolated, effects-disabled test environment; pin container/image or source commit, database, runtime and config.
3. **Protocol probe:** verify capture/query routes, required headers, declared version range, accepted media types, response media types, unsupported-type responses, pagination, named queries, authorization and documented limits.
4. **Normative validation:** run the exact source document through Draft-07 JSON Schema, JSON-LD expansion and SHACL; preserve separate receipts.
5. **Semantic round-trip:** run the frozen ObjectEvent, TransformationEvent and AggregationEvent corpus through export/import across the independent server pair; compare every supported field and retain unsupported fields opaquely or return explicit Unsupported/Indeterminate.
6. **Adversarial controls:** duplicate IDs/replay, invalid CBV, bad time/offset, unsupported extension, changed subject, broken transformation/aggregation lineage, corrected/error-declared events and unauthorized disclosure.
7. **Performance only after correctness:** use #5395's 1k/10k/50k event sizes and at least 30 repetitions per condition; report p50/p95, throughput, CPU, peak RSS, errors, resource cost and integration effort. No performance score may hide a semantic-loss or hard-gate failure.

A server that supports only a subset can participate in a clearly labelled compatibility comparison for the intersection profile. It cannot be upgraded in the report to full standard conformance. Arms that cannot perform the frozen task remain visible as unsupported/ineligible with the reason.

## 5. What this changes in #6105

- #6105 owns the broad mapping from Mycelix checkpoint semantics to NIST/GS1/W3C/IETF profiles and the 20 synthetic development scenarios.
- #5395 owns GS1 normative artifact intake, Draft-07/JSON-LD/SHACL validation, GS1 event fixture conformance and the independent server baseline/performance harness.
- FasTnT and OpenEPCIS are candidate external systems, not validated conformance results or formal partners.
- OpenEPCIS validator services are same-family diagnostics, not independent judges.
- The Mycelix mapping/adapter remains a candidate until source, behavior, round-trip and evidence-preservation gates pass.

## 6. Status and non-claims

This register was produced by public-source and source-code inspection only. No server was deployed, no API request was sent to either candidate, no license compatibility review was performed beyond identifying declared Apache-2.0 files, no normative JSON-LD/SHACL validation was executed and no throughput or semantic-fidelity measurement was taken.

It does not claim GS1 endorsement, server conformance, Mycelix interoperability, external partnership, physical-event truth, product authenticity, regulatory compliance, NIST conformity or SOTA performance.


### 2026-11-14 exact-source recheck

The official EPCIS 2.0.0 artifact page and exact Draft-07 JSON Schema are browser-renderable, but raw bytes, response headers/final URL and SHA-256 remain uncaptured. Context and SHACL still fail only at browser media-type handling. A `gs1/EPCIS` mirror context at commit `6f714682` is explicitly labelled a v2.1 public review draft; it is not substituted for the normative 2.0.0 URL. Keep #5395 raw-byte intake and all three official validator lanes open.
