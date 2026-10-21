# Mycelix Organization Adoption Portfolio v1

**Registry ID:** `MYC-ORG-PORTFOLIO-001`  
**Snapshot date:** 2026-10-21  
**Status:** research/profile contract; no partner relationships asserted  
**Parent issue:** [MYC-ORG-PORTFOLIO-001 #5388](https://github.com/Luminous-Dynamics/mycelix/issues/5388)  
**Related general pilot catalog:** [WEB-007 #1039](https://github.com/Luminous-Dynamics/mycelix/issues/1039)

## Why this exists

The monorepo's target-organization research spans supply-chain traceability, identity, decentralized science, environmental finance, agentic operations and standards. Mycelix already has many domain-specific work items, but a prospective adopter needs a narrow proof-of-value path, a clear statement of what remains external, measurable success conditions, and a safe exit—not a pitch to adopt the whole platform.

This portfolio connects selected public organizations, protocols and registries to existing Mycelix work. It is a **technical integration map**, not an outreach tracker. The public artifact intentionally excludes private contact history and does not imply that a named organization is a partner, customer, design partner, adopter, or endorser.

**Source-freshness guard:** 28 of 29 machine profiles currently set `source_evidence_state: reference_links_only`, `source_snapshot_manifest_path: null`, `source_snapshot_manifest_sha256: null`, and `source_snapshot_required_before_pilot: true`. The Integral Collective specifications profile is the first exception: it pins the official repository to commit [`402bf3ed46779f81ba720b3f4dabfe0cb1ab1bc5`](https://github.com/Integral-Collective/integral-specifications/commit/402bf3ed46779f81ba720b3f4dabfe0cb1ab1bc5) via [its source snapshot manifest](./source-snapshots/integral-collective-specifications-v1.json), whose SHA-256 is `cd7148a62c2ce63f874a1289de66227d3bff31b9936a96593aa7b9bd93bbb2fd`. This pins repository revision metadata only; raw source bytes are not copied, and no claim is made that any external spec is ratified or authoritative. The source snapshot schema is [versioned here](./org-adoption-source-snapshot-v1.schema.json). The catalog verifier rejects a claimed attachment if its manifest is missing, its digest differs, its URL set is incomplete, its source is not revision/content-qualified, or captured source bytes mismatch. A URL that still resolves does not establish that its content or status is unchanged.

## Design rules

1. **One profile, one bounded workflow.** Identify the exact workflow and the smallest pilot that produces useful evidence.
2. **Compose existing owners.** Do not create parallel identity, authority, evidence, finance, logistics, PID, or optimizer kernels.
3. **Keep institution and action authority separate.** Legal-entity identity, person's identity, role evidence, local delegation, exact-action authorization, and external effect are separate propositions.
4. **Keep proposals and effects separate.** A plan, record, model result, authorization, dispatch, provider acknowledgement, settlement, delivery, and accepted outcome are not synonyms.
5. **Map external meaning lossfully.** Every external field/status mapped into Mycelix must preserve source/profile/version, units, valid time, provenance, and unsupported/unknown dispositions.
6. **No inferred relationships.** The JSON uses `relationship_status: not_recorded` deliberately. Public URLs are source references, not evidence of contact or partnership.
7. **No claim inflation.** Schema validation establishes catalog conformance only; it does not establish an adapter, real-world deployment, legal compliance, efficacy, or endorsement.
8. **Snapshot before execution.** The URLs below were checked on 2026-10-21 for this research pass, but URLs are mutable. A real pilot must retain the actual source bytes/revisions and digests, collection time, and license/use decision in its source manifest.

## Candidate portfolio

| Priority | Organization / reference | Smallest useful Mycelix proof of value | Existing Mycelix owners / gap |
|---|---|---|---|
| P0 | Common Planet Foundation / Creditism | Frozen synthetic event ledger that separates Personal Credit issuance/deletion, Community Credit allocation/spending, contribution recognition, Bonus and physical capacity. Compare allocation mechanisms with the same capacity inputs. | #4572, #4573, #4771, #5366. Do not imply AYU Points are money or that a live Creditist economy exists. |
| P0 | Integral Collective specifications | Reproducible source manifest for the exact public source generations and their declared statuses; replay a decision → authorization → execution-attempt → observation → review trace without promoting draft sources into ratified contracts. | #3213, #3336, #5366. Source drift flags compatibility; it does not rewrite adapters or decide external authority. |
| P1 | Holochain ecosystem | One bounded cross-hApp workflow with explicit capability/authority checks, offline/partition behavior, data export and independent recovery. | #1039 and existing bridge/identity/authority work. Platform capability is not evidence of a Mycelix deployment or endorsement. |
| P1 | TraceLink | Map one synthetic track-and-trace flow into a loss-aware event projection, retaining identifiers, timestamps, source, correction lineage and distinctions between shipment, receipt and acceptance. | #437, #4901, #1112. No TraceLink-specific adapter or customer relationship is asserted. |
| P1 | TRASCE consortium | Demonstrate multi-party provenance exchange for a synthetic ingredient/packaging chain while retaining each participant's assertion, source, evidence status and contested/missing links. | #437, #4901, #1112. Do not infer supplier links or sustainability outcomes from membership or proximity. |
| P1 | OPTEL Group | Test a traceability/export adapter seam against a synthetic product-lifecycle record; compare the source events with Mycelix's normalized view and identify every lossy mapping. | #437, #4901, #1112. No OPTEL-specific integration is asserted. |
| P1 | VitaDAO | Bind a synthetic research-funding project to stable researcher, organization, project, output and award identifiers; separate funding decisions, evidence of research progress and independent replication. | #2418, #3327, #4950. Token or community activity is not proof of research validity. |
| P1 | Molecule | Map a synthetic research proposal through sponsor terms, milestones, evidence, updates and IP/rights references without treating funding as scientific validation. | #2418, #3327, #4950. Do not assume one ownership/legal model applies to all projects. |
| P1 | ResearchHub | Preserve separate identities for research outputs, proposals, funding, reviews and recognition; test that a reward or review event does not silently become a claim of scientific truth. | #2418, #3327, #4950. Reputation/reward is not a substitute for methods or independent reproduction. |
| P1 | GLEIF / LEI and vLEI ecosystem | Consume a sample legal-entity/representative-role credential as evidence, then require local recognition plus an exact-action authority grant before a consequential operation. | #1070, #1112. Identity/role evidence alone must not authorize a payment or contract. |
| P1 | W3C Verifiable Credentials | Test a narrow VC 2.0 mapping with issuer, holder, verifier, status/currentness, privacy and profile/version preserved. | #1070, #1112. VC data-model compatibility is not trust in an issuer or legal recognition. |
| P1 | ORCID | Bind a researcher-controlled ORCID identifier to a local researcher identity through an explicit, attributable mapping; preserve source freshness and local correction. | #2418. Mycelix DID and ORCID are not interchangeable identifiers. |
| P1 | Research Organization Registry (ROR) | Resolve a research/funding organization reference without replacing the local institutional record or silently overwriting local assertions with imported metadata. | #2418. A ROR match alone does not prove signatory authority or a funding relationship. |
| P2 | Toucan Protocol / climate-credit provenance | Prototype an evidence-lineage packet for a synthetic environmental-credit record: origin, methodology, registry/project/vintage references, revisions, retirement/transfer events where present, and unknowns. | #1112 and #2922 provide adjacent semantics/evidence; #5390 tracks the missing methodology-specific profile. |
| P1 | Catena Labs / agentic financial operations | Simulate an agent's vendor-payment proposal: bind agent and principal, vendor identity, policy version, amount limits, approval, execution receipt and reconciliation. Include stale-policy and replay controls. | #1072, #3650, plus existing agent-authority/Finance work. Agent identity or an audit log is not authority to move money. |

### Additional profiles recovered from the monorepo's target-organization research

The machine-readable roster also includes these named target-list organizations. They are deliberately separate profiles: they exercise different evidence sources and failure modes even when they sit in the same vertical.

| Organization | Bounded proof-of-value | Existing owner seam |
|---|---|---|
| Antares Vision Group | Serialization/aggregation and inspection-to-receipt event mapping | #437, #4901, #1112 |
| Acviss | Product-authentication observation vs physical provenance and warranty decision | #437, #4901, #1112 |
| Authentix | Brand-protection claim provenance, counterfeiting/diversion dispute handling | #437, #4901, #1112 |
| Transmute | Credential/data exchange with issuer trust, status and local authority kept distinct | #1070, #1072, #1112 |
| Verra | Methodology- and registry-bound environmental-project evidence; research-only until a domain-specific contract is qualified | #1112, #2922, #5277, #5390 |
| KlimaDAO | Token/market event vs underlying methodology and retirement/impact evidence; research-only until a domain-specific contract is qualified | #1112, #2922, #4572, #5390 |
| Ocean Protocol | Purpose-bound data access request, compute/disclosure mode, rights and output lineage | #2923, #2970, #1112 |
| AxonDAO | Participant-consented research-data use and independent research-result evidence | #2418, #3327, #4950, #2970 |
| TransCrypts | Holder-authorized employment/income credential sharing with source freshness and purpose scope | #1070, #1112, #2418 |
| Digital Bazaar | VC proof/issuer/source fidelity across translations and selective disclosure | #1070, #1112, #1072 |
| Decentralized Identity Foundation (DIF) | DID/VC method and profile mapping without inferring trust or authority | #1070, #1112 |
| Danube Tech | Resolver/provider evidence and versioned DID/VC resolution receipts | #1070, #1112 |
| Marblism | Effects-disabled agent workflow with principal binding, consent, tool scope, approval and action receipts | #1072, #3650, #2939 |
| GS1 EPCIS / CBV | Preserve a specific ratified event-model version; test loss-aware mapping of scan/shipping/receiving/acceptance/correction events without inventing a parallel vocabulary | #437, #1112, #4901 |

Climate-credit profiles are explicitly marked `gap_requires_issue` and tracked by [CLIMATE-PROV-001 #5390](https://github.com/Luminous-Dynamics/mycelix/issues/5390): existing evidence/provenance owners are adjacent, but they are not a substitute for a methodology- and registry-specific contract. Those profiles must remain research-only and cannot emit qualified impact claims.

### Official public source references

- Common Planet, [Creditism architecture](https://common-planet.org/creditism/architecture) and [Creditism overview](https://common-planet.org/creditism)
- Integral Collective, [integral-specifications](https://github.com/Integral-Collective/integral-specifications)
- Holochain, [developer portal](https://developer.holochain.org/) and [get involved](https://www.holochain.org/get-involved/)
- GS1 EPCIS/CBV, [official standards page](https://www.gs1.org/standards/epcis)
- TraceLink, [official site](https://www.tracelink.com/)
- TRASCE, [2025 annual report](https://trasceconsortium.com/wp-content/uploads/2025/12/TRASCE_Annual-Report_2025.pdf)
- OPTEL, [official site](https://www.optelgroup.com/en/)
- VitaDAO, [research](https://www.vitadao.com/research)
- Molecule, [official site](https://www.molecule.to/)
- ResearchHub, [official site](https://www.researchhub.com/)
- GLEIF, [verifiable LEI (vLEI)](https://www.gleif.org/en/organizational-identity/lei-vlei/the-verifiable-lei-vlei)
- W3C, [Verifiable Credentials Data Model v2.0 Recommendation](https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/)
- ORCID, [official site](https://orcid.org/)
- ROR, [official site](https://ror.org/)
- Toucan, [documentation](https://docs.toucan.earth/)
- Catena Labs, [official announcement and agent commerce kit context](https://catena.com/blog/announcing-catena-labs)
- Antares Vision Group, [official site](https://antaresvisiongroup.com/)
- Acviss, [official product/traceability information](https://acviss.com/products/)
- Authentix, [official brand-protection solutions](https://authentix.com/brand-protection-solutions/)
- Transmute, [official decentralized-data guide](https://guide.transmute.industries/verifiable-data-platform/learning-center/decentralized-101)
- Verra, [official standards organization](https://verra.org/)
- KlimaDAO, [official app](https://app.klimadao.finance/)
- Ocean Protocol, [official overview](https://docs.oceanprotocol.com/discover/what-is-ocean)
- AxonDAO, [official site](https://www.axondao.io/)
- TransCrypts, [official site](https://www.transcrypts.com/)
- Digital Bazaar, [official site](https://www.digitalbazaar.com/)
- Decentralized Identity Foundation, [official site](https://identity.foundation/)
- Danube Tech, [official site](https://www.danubetech.com/)
- Marblism, [official site](https://www.marblism.com/)

These references document public products, organizations or standards only. They do not mean that Mycelix has contacted or integrated any of them.

## Portable proof-of-value bundle

The profile catalog is the *discovery index*, not proof of an adapter. A concrete pilot should emit the separate [pilot proof-bundle schema](./org-adoption-pilot-bundle-v1.schema.json) and start from the [synthetic supply-chain example](./org-adoption-pilot-bundle-example-v1.json). The [cross-invariant verifier](./verify-org-adoption-pilot-bundle.cjs) is zero-dependency. The reference example is marked `contract_example_only`: its metrics are `not_measured`, negative controls are `not_executed`, both implementations are `not_executed` with null digests, and export/rollback are not claimed as performed. The verifier generates an external receipt; the input bundle does not self-attest its own result.

Run from this directory:

```sh
# Verify the supplied bundle and emit a digest-bound receipt
node verify-org-adoption-pilot-bundle.cjs

# Separately execute the 12 internal mutation controls
node verify-org-adoption-pilot-bundle.cjs --self-test

# Repeatable scale runs with portable measurements and artifact digests
node benchmark-org-adoption-pilot-bundle.cjs --runs 5 --sizes 1000,10000,50000 --out /tmp/mycelix-org-adoption-scale.json
```

The default verifier checks relationships JSON Schema alone cannot conveniently express: source/record linkage, manifest and synthetic source-content digests, baseline/candidate input equality, equal resource and authority-budget digests, evidence-reference closure, metric result arithmetic, explicit measured-versus-unmeasured state, export and rollback disposition, effects-disabled mode, and relationship claim guard. The optional `--self-test` mode mutates 12 hostile variants and requires each to be rejected. Those are **verifier mutation tests**, not executed domain-specific pilot controls. The receipt reports how many domain controls and metrics were actually executed/measured, and does not promote the reference example into a measured evaluation.

The companion `benchmark-org-adoption-pilot-bundle.cjs` generates fixtures up to the schema's 50,000-event cap, runs the fast verification lane in fresh Node processes, and emits a machine-readable report with event count, input/bundle/evaluator digests, samples, median and max-of-N, and runtime conditions. Its timings deliberately exclude schema-instance validation and the 12-mutation self-test lane; run those as separate checks.

JSON Schema validation is a separate lane; the zero-dependency evaluator does not pretend to be a general Draft 2020-12 implementation. The example is a contract witness only. It does not execute either named implementation, run a vendor integration, or prove the scenario happened in the physical world.

## Benchmarking against industry and state of practice

- **Adopt established event semantics where they fit.** GS1 EPCIS/CBV 2.0 already provides standardized supply-chain event representations, sensor-data support and REST interfaces. Mycelix should preserve EPCIS event meanings and provenance, and add value in cross-organization evidence qualification, contradiction handling, rights/authority boundaries, and portable replay—not invent a parallel traceability vocabulary. Reference: [GS1 EPCIS](https://www.gs1.org/standards/epcis).
- **Adopt mature measurement discipline.** OpenTelemetry's SDK performance guidance specifies configuration and measurement practices, including warm-up where relevant. Use repeated runs and disclose hardware/software/input conditions instead of treating one latency number as a benchmark. Reference: [OpenTelemetry performance benchmark guidance](https://github.com/open-telemetry/opentelemetry-specification/blob/main/specification/performance-benchmark.md).
- **Tailor evaluation to the actual mission.** NIST's initial public draft of AI 200-2, TEVV-Athlon, structures assessments around organizational objectives, measurement concepts, events and tools. Use the idea of a purpose-specific evaluation plan, while treating this as a draft framework—not a certification or endorsement. Reference: [NIST AI 200-2](https://doi.org/10.6028/NIST.AI.200-2.ipd).

For every vertical, baseline and candidate must receive the same frozen input digest, resource budget, authority budget, measurement window and declared measurement profile. Report an outcome vector—not a single universal score:

- semantic fidelity: preserved known fields, explicit unsupported fields, and round-trip loss;
- correctness: false claim promotions, missing/duplicate events, replay acceptance, and disputed-state preservation;
- trust boundaries: stale authority, wrong subject, source drift, rights gaps and cross-organization attribution;
- operational performance: throughput and p50/p95/max latency under declared conditions, peak memory, operator effort, integration cost and recovery time;
- portability: reproducible receipt, full export, independent replay and tested rollback/off-ramp;
- consequential outcomes: only after real deployment, a suitable baseline, predeclared evaluation plan and permission to use the required data.

### First local reference measurement (2026-10-21)

A small reference run was performed with Node v22.16.0 in a Linux x86_64 container (AMD EPYC 9V74 host, 3 visible vCPUs, 2-CPU cgroup quota, 4 GiB memory limit). Five independent process launches were measured for each size in `cross-invariant-only` mode, against synthetic contract-example bundles that explicitly report no measured application outcomes. The schema-marker argument intentionally isolated the Node cross-invariant verifier; these timings do not include full JSON Schema evaluation.

| Synthetic events | Bundle bytes | Median wall time | Maximum of 5 runs |
|---:|---:|---:|---:|
| 1,000 | 386,905 | 50.37 ms | 53.35 ms |
| 10,000 | 3,822,658 | 154.33 ms | 170.01 ms |
| 50,000 | 19,132,658 | 593.43 ms | 699.26 ms |

At 50,000 events, one local `--self-test` run including the 12 verifier mutation variants took 7.02 s and peaked at about 503,724 KiB RSS. The five-run maximum is **not a statistically reliable p95**; these values are a reproducible starting point, not a vendor/SOTA comparison. A subsequent five-run refresh of the local verifier mirror produced medians of 50.37 ms, 154.33 ms and 593.43 ms, respectively, with maxima 53.35 ms, 170.01 ms and 699.26 ms. This is a local mirror of the evolving branch, not bit-for-bit evidence that the current remote head was executed; exact-head rerun remains required. The source-content digest scan was changed to use indexed event IDs rather than repeated list scans before these fast-lane measurements.

Proposed regression ceilings for this same harness/host are ≤90 ms at 1,000 events, ≤220 ms at 10,000, and ≤950 ms at 50,000 for cross-invariant-only mode; ≤10 s for a 50,000-event `--self-test` run. These are provisional local regression gates, not universal performance claims. Re-run at least 30 trials for percentile claims, pin the CPU/memory and runtime conditions, and compare against an independently implemented baseline before describing the system as faster than a competitor.

Suggested scale gates are *targets to measure, not achieved results*: first validate the synthetic 4-event contract example; then add a measured pilot packet with executed baseline/candidate hashes and domain controls before scaling to 1,000, 10,000 and 50,000-event corpora with increasing event-type and source diversity. The v1 bundle schema caps one manifest at 50,000 events; larger corpora require explicit chunking/streaming semantics before raising that bound. Hold exactness and claim ceilings constant while measuring scale. The example's five declared-but-unexecuted domain controls and 12 verifier mutation cases do not demonstrate industrial readiness or SOTA performance.

## Acceptance profile for every pilot

Before implementation work is proposed, run `node verify-org-adoption-profiles.cjs` from this directory. It checks cross-row invariants and emits a digest-bound JSON receipt. Validate the catalog instance against `org-adoption-profiles-v1.schema.json` separately with a Draft 2020-12 JSON Schema validator; the zero-dependency Node script does not claim to replace that validation.


Every concrete pilot derived from this portfolio must provide:

- **Frozen input manifest:** source identity/revision/digest, schema/profile, units, valid/observed time, source rights, synthetic/real label, and known gaps.
- **Baseline:** the current/manual process or a qualified conventional reference, evaluated against the same inputs and authority budget.
- **Outcome vector:** correctness, latency, staff effort, data completeness, correction/appeal behavior, reliability, and distribution of benefits/harms where relevant. Do not collapse unlike outcomes into one universal score.
- **Negative controls:** stale/revoked credential; wrong entity or representative; conflicting event; duplicate/replayed event; unknown relationship; bad units; changed source revision; missing evidence; policy change after approval; failed external effect.
- **Operational boundary:** read-only / effects-disabled by default; human approval for consequential actions; auditable export; provider exit and rollback exercise.
- **Claim ceiling:** state exactly what was verified and what remains untested.

## Suggested sequencing

1. **Finish P0 reference seams first:** Common Planet mechanism semantics and Integral source-generation traceability. They already have owning issue tracks and can produce inspectable fixtures without inventing another economic or governance engine.
2. **Use organizational identity as shared substrate:** GLEIF/vLEI and W3C VC profiles must compose local institutional authority rather than become a universal authority source.
3. **Run one supply-chain and one DeSci vertical in parallel:** they exercise cross-organization provenance using different event semantics, and expose where the shared envelope works versus where domain-specific fields must remain.
4. **Run the agentic-payment simulation effects-disabled:** test principal binding, exact action, policy currentness, external receipt, and reconciliation before any live-money experiment.
5. **Keep the climate-credit profile at research-only until the methodology/registry-specific contract is reviewed.**

## What success means

A passing catalog validator proves only that this roster conforms to its declared schema. The current catalog has 28 reference-only profiles and one immutable-revision pin for the Integral repository; neither status makes a draft interface ratified or establishes external authority. Each pilot must separately freeze and qualify its relied-upon sources, pass exact-head tests, and pass its domain-specific oracle. An external claim such as “Mycelix improves traceability,” “reduces reconciliation cost,” or “is more effective than the incumbent” requires a paired baseline, a prespecified measurement plan, retained negative results, and a suitably independent evaluation.

No target organization is treated as a benchmark owner or endorser unless a real, sourced agreement establishes that role.
