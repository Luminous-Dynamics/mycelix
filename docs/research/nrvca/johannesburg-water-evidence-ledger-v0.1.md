# Johannesburg Water Service Evidence Ledger v0.1

**Status:** source discovery and official reported assertions only  
**Fixture:** [Machine-readable source ledger](../../../fixtures/nrvca-johannesburg-water-source-ledger-v0.1.json)  
**Rust tests:** `mycelix-workspace/tools/nrvca-validator/tests/johannesburg_water_evidence_ledger.rs`  
**Tracking issue:** [NRVCA-012 / #5431](https://github.com/Luminous-Dynamics/mycelix/issues/5431)

## Why this is a separate evidence dimension

National accounts describe monetary economic flows and related stocks; they do not directly measure whether a household received safe water at sufficient pressure for enough hours. NRVCA therefore needs service-outcome evidence without treating a press release or outage notice as if it were telemetry.

The source-discovery pass found three distinct classes of official material:

- Johannesburg Water's interruption page separates unplanned interruptions, planned interruptions and completed maintenance/repairs: https://www.johannesburgwater.co.za/interruptions/
- Johannesburg Water's daily water notices page is accessible as a web page, but the latest visible notice assets found in this retrieval were images labelled with 15 October 2026 dates rather than a structured incident list: https://www.johannesburgwater.co.za/daily-water-notices/
- The DWS Gauteng Water Security Dashboard index is discoverable and search-indexed through a monthly update dated 31 May 2026, but direct page retrieval returned 403 in this pass: https://www.dws.gov.za/gautengreports.aspx

Two South African Government statements on the January–February 2026 disruptions provide official context:
- Gauteng Provincial Government, 13 February 2026: https://www.gov.za/news/media-statements/gauteng-provincial-government-addresses-water-challenges-13-feb-2026
- The Presidency / Water Task Team, 13 February 2026: https://www.gov.za/news/media-statements/deputy-president-paul-mashatile-convenes-water-task-team-address-gauteng
- Department of Water and Sanitation statement, 19 February 2026: https://www.gov.za/news/media-statements/minister-pemmy-majodina-permits-rand-water-additional-abstraction-stabilise

The fixture records six source locators and three source-grounded reported assertions. It does not claim that the underlying image/report bytes were acquired or hashed. The government statements' quantities and descriptions remain **reported assertions**, not imported sensor series.

## Paradigm shift: evidence graph, not flat event table

A flat incident row often collapses several distinct questions: Did someone publish a notice? Did the asserted event happen as described? Which areas and times did it affect? Was supply actually delivered? Was it safe and adequate? How many people experienced interruption? Was an intervention responsible for recovery?

NRVCA represents those as a chain with independent evidence gates:

```
publication/notice
      |
      v
source-attributed reported assertion
      |
      +----> operator measurements (not yet acquired)
      |
      +----> independent/community corroboration (not yet acquired)
                      |
                      v
           qualified service-outcome estimate
```

The chain may have missing edges. A notice is evidence that an organization published a notice. It is not, by itself, proof of a customer-level interruption or restoration. A completed-maintenance message is not proof that water service has returned at every affected connection. Planned interruptions and emergency incidents must remain separate categories.

## Current reported assertions and limits

The source ledger retains three types of contextual assertion:
1. Official reporting on contributing factors and affected parts of Gauteng in February 2026. The statement's causal narrative is preserved as reported context, not treated as an estimated causal decomposition.
2. An official report that Rand Water returned to normal bulk treated-water supply of 5,000 million litres/day by 4 February 2026. This is a source-reported quantity—not an independently verified continuous flow series and not the volume delivered to households.
3. Official reporting that municipal reservoir storage remained depleted or constrained in some areas. The fixture has no site-level reservoir readings, exact start/end interval, or local geometry.

Some of these statements repeat or summarize shared official briefings. They are not independent measurements simply because they appear in separate publications.

## Mandatory data contract

Each source must record publisher, locator, publication/retrieval dates where known, medium, source-byte acquisition status, digest or explicit null, rights status, scope, accessible content and forbidden inferences.

Each reported assertion must record:
- linked source IDs and assertion class;
- publication date separately from event valid time;
- optional start/end with explicit null for unknown times;
- geographic text separately from geocoded geometry;
- values, units and semantics when quoted by the source;
- whether the quantity is official reporting, operator measurement, or independent observation;
- outcome status that does not exceed the evidence.

Unknown values remain unknown; do not fill gaps with zero or inferred durations. Similar place names do not establish that two notices describe the same incident. Corroborating publications should attach to a shared asserted event when identity is supported, not inflate event counts.

## Benchmark programme: acceptance targets, not measured achievements

| Benchmark | Acceptance target |
|---|---|
| Provenance coverage | 100% of claims link to at least one source with publisher and locator |
| Byte-integrity truthfulness | 100% of unacquired assets retain a null digest; no fabricated source hashes |
| Evidence-tier separation | Zero press statements/notices promoted to operator telemetry |
| Time integrity | Publication time distinct from event start/end; all unknown bounds explicit |
| Geography integrity | 100% of unverified locations remain reported text without invented geometries |
| Event de-duplication | Multiple sources can corroborate one event without counting it twice |
| Status semantics | Planned, unplanned, completed maintenance and confirmed restoration are separate states |
| Service-outcome validity | Customer-minutes lost, affected-household counts, water quality and restoration time are reported only with adequate supporting evidence |
| Freshness | Report/index last-visible date is explicitly a source-coverage observation, not proof no later report exists |

The next comparative study should assess source completeness and source-to-source disagreements against a manually reviewed sample, before attempting outage prevalence or customer-minutes lost. Once access is authorized, pair operator reservoir/flow/pressure/quality telemetry with incident notices and a privacy-protective independent service-outcome sample. Pre-register geography, missingness, sampling, de-duplication, latency, and uncertainty rules first.

## What would beat current practice

The credible ambition is not a fancier dashboard. It is a reproducible lineage from the original source asset to a dated claim to a qualified service indicator, with explicit unknowns and a reviewable difference between reported and measured evidence.

Targets for a qualified v1:
- every retained source file pinned by digest and retrieval metadata;
- two independent extraction/corroboration paths with disagreement logs;
- a historical incident register with explicit coverage denominator and publication-lag metrics;
- service indicators stratified by area and relevant groups where evidence and privacy safeguards support this;
- no outage-frequency or causal-impact claim unless the underlying event/telemetry data support it;
- held-out incident review and external operator/community review before any automated recommendation workflow.

This is not a live water alert service and does not report water availability in Johannesburg on 22 October 2026. The present work is a provenance and evidence-boundary prototype only.

