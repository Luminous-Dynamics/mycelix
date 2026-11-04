# Cross-repository integration map

This is a coordination design, not evidence that runtime adapters already exist.

| Workstream | Receipt/object | Critical rule | Next step |
|---|---|---|---|
| [Symtropy Antarctic energy profile PR #2687](https://github.com/Luminous-Dynamics/symtropy/pull/2687) | Scenario profile and run receipt | Four synthetic periods are fixtures, not plant measurements or equipment sizing | Adopt common receipt; hash permitted source artifacts; compare incumbent, rules, LP/MPC and robust methods on locked holdouts |
| [Sol Atlas Antarctic manifest PR #305](https://github.com/Luminous-Dynamics/sol-atlas-leptos/pull/305) | Site manifest and observation provenance | Preserve AADC conflicts; do not assign annual aggregates to station pins; ERA5 is gridded reanalysis, not site measurement | Typed temporal/spatial source records and claim labels; CRS/aggregation mutation tests |
| [Symthaea Larsen audit PR #8444](https://github.com/Luminous-Dynamics/symthaea/pull/8444) | Versioned assessment-table transcription | Table audit only, not reproduction of the geological model or distributions | Attach primary artifact digests after source acquisition/stewardship review; independently audit units/quantiles |
| Mycelix | Receipt hash and verifier attestation | DHT may store/refer to assertions but cannot prove scientific truth or reuse permission | Build portable adapter after schema, signing, privacy and key recovery are reviewed |

Ownership: domain repos own their science/physics, validators, metrics, limits and display semantics. Evidence Plane owns shared receipt shape, tier/gate meanings, digest rules, lineage relationships and conformance fixtures. Mycelix later owns authorship/attestation and optional anchoring; raw datasets should not be copied into Holochain entries by default.

Consumers may not upgrade a tier/claim ceiling. Consumers validate a pinned schema or reject it. Displays preserve data kind, uncertainty, time/space support and citation. Cross-repo receipts use exact commit and content digest; branch links are supplementary. Migrations emit new versions rather than rewriting history.

Status snapshot for 2026-10-22: the three domain PRs are open drafts. Their dedicated workflows were queued/pending, and some broad CI checks were skipped on drafts. None is a CI pass or field/scientific validation. The source workstreams have not ingested the raw historical AADC files or raw USGS release at this snapshot; recheck heads and jobs before relying on it.
