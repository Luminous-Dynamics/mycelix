# Finance Platform Adapter Envelope v1.0.0 — S0 draft

This package defines a vendor-neutral, **read-only** envelope for preserving evidence from existing privacy, data-governance, accounting-close and financial-reporting systems.

It is not a live connector, vendor export format, vendor certification, privacy-law engine, accounting ledger or authority to take business actions. The ten fixture cases are synthetic adapter projections authored for conformance testing; no payloads were obtained from OneTrust, BigID, Microsoft Purview, BlackLine or Workiva.

## Why this layer exists

Incumbent products already provide substantial operational depth:
- OneTrust documents processing inventories, data maps, RoPA generation and directional transfer/contextual relationships. Its transfer relationship feature guide currently says transfers to people or parties with access are not supported in that feature and custom relationships are not yet available. That creates a specific access-principal preservation test, not proof that the entire product cannot represent access.
- BigID documents broad discovery/classification across structured and unstructured sources and data/AI inventories. Mycelix should consume scan/classification observations with their source, scope and confidence, not rebuild a broad discovery estate.
- Microsoft Purview Data Map combines metadata discovery, scanning/classification and lineage, but Microsoft's source matrix shows capabilities vary by connector. A registered connector is not proof that classification, sensitivity labels, policies and lineage are all available for that source.
- BlackLine documents reconciliation, transaction matching, journal automation/posting and close workflows. Mycelix should ingest reconciliation evidence and preserve authority boundaries, not build a shadow posting system.
- Workiva documents source-linked SEC reporting, XBRL, collaboration, permissions, version control and audit trails. Mycelix should preserve source-to-disclosure lineage, not rebuild a filing authoring product.

Primary documentation:
- OneTrust mapping: https://docs.onetrust.com/en/privacy-professional/documenting-data-flows-and-processing-activities/data-mapping-automation-overview
- OneTrust contextual transfers: https://docs.onetrust.com/en/privacy-automation/data-mapping-automation/configuring-data-transfers-and-contextual-relationships/transfers-and-contextual-relationships-feature-guide
- BigID discovery/classification: https://bigid.com/solutions/discovery-classification/
- Microsoft Purview Data Map: https://learn.microsoft.com/en-us/purview/data-map
- Microsoft Purview source capability matrix: https://learn.microsoft.com/en-us/purview/data-map-data-sources
- BlackLine Financial Close: https://www.blackline.com/products/financial-close/
- Workiva SEC Reporting: https://www.workiva.com/solutions/sec-reporting/

## Paradigm

Mycelix should become an **evidence qualification and temporal replay plane**, not one more enterprise catalog.

The direction is:

source observation -> immutable source identity/revision -> typed objects and directed relationships -> capability and field-loss accounting -> bitemporal evidence filter -> deterministic review disposition -> human-authorized action in the system of record

Reuse standard provenance and lineage concepts instead of inventing another disconnected graph:
- W3C PROV-O provides Entity, Activity and Agent concepts and derivation/attribution relationships: https://www.w3.org/TR/prov-o/
- OpenLineage defines run, job and dataset entities, events and extensible facets: https://openlineage.io/docs/spec/

These standards complement rather than replace the finance-specific envelope. OpenLineage lineage is not itself legal transfer evidence; PROV provenance is not an authorization decision.

## Contract

The finance platform adapter envelope schema captures:
- source system family, tenant namespace, connector, export revision, observed time, capture completeness and a payload digest slot;
- explicit capability coverage states: supported, unsupported, partial, not scanned, not configured, inaccessible, truncated or unknown;
- external object IDs in a tenant/namespace plus provider revision;
- directed source-to-target relationships;
- processing activity, purpose, data-element, access-principal, evidence and upstream-origin references at relationship level;
- valid-time and recorded-at time;
- field-level mapping status: exact, normalized, unmapped, unsupported or ambiguous;
- an empty effects array. This S0 contract is read-only by design.

Never silently convert unsupported, unscanned, inaccessible, truncated or unknown to supported, complete, local or absent. A missing access principal is not evidence that no person or service account had access. A capability can vary by source scope. Preserve source revision and namespace in object identity; an external ID is not globally unique.

All timestamps use normalized RFC 3339 UTC strings for deterministic lexical ordering in this draft. Implementations must reject timezone-normalization ambiguity before relying on lexical comparison. Relationship valid-time and source recorded-at/knowledge time remain separate.

## S0 fixture matrix

The candidate and oracle fixtures contain ten public synthetic cases:

1. OneTrust-like transfer mapping where access-principal coverage is explicitly unsupported.
2. OneTrust-like subprocessor relationship preserving direction and a person access principal.
3. BigID-like classification coverage without implying transfer authority.
4. Purview-like partial lineage and an ambiguous column-mapping field.
5. BlackLine-like reconciliation evidence while journal posting remains outside the adapter's authority.
6. Workiva-like disclosure linkage back to a journal source and XBRL tag.
7. A future-recorded relationship excluded by an earlier requested knowledge frontier.
8. A truncated export with explicit pagination incompleteness.
9. Two export copies tied to one upstream origin, which must not count as independent corroboration.
10. Reused external object IDs in two namespaces, which must remain distinct.

These examples are **not** native vendor payloads or reverse-engineered private APIs. They are provider-neutral test projections. The OneTrust gap is grounded in its published feature guide; BigID/Purview/BlackLine/Workiva examples test capability boundaries suggested by their public product documentation.

## Local validation

From the rust-resolver directory:

```sh
cargo test --locked --bin evaluate_platform_adapter_conformance
cargo run --locked --bin evaluate_platform_adapter_conformance
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
```

The Rust binary validates the candidate bundle, public oracle, each envelope and selected mapping invariants: unique case IDs, relationship endpoint references, direction, bitemporal relationship visibility, field-mapping loss, incomplete capabilities, access-principal refs, upstream-origin refs and no external effects.

The public oracle also includes proposed adapter-state and reason-code expectations for a future higher-level conformance evaluator; this initial Rust validator does not yet calculate those composite states/reasons. A separate static JavaScript audit compared the authored reason labels and core mappings; that is not an independent Rust run.

## Targets (proposed, not achieved)

- 100% preservation of external identity, namespace, revision and relationship direction for declared supported fields.
- At least 99% precision and recall for field-level purpose/data-element mappings on an expert-adjudicated set.
- At least 99% recall for declared supported location/access/backup/onward facets; zero silent loss of access-principal relationships.
- 100% of unsupported, unscanned, inaccessible, partial and truncated states explicitly surfaced.
- 100% identical replay for frozen source/policy/knowledge frontiers.
- 100% stale-revision invalidation and correlated-origin negative-control detection.
- Zero external mutations by the adapter.
- Zero critical false qualification or false assurance on seeded adversarial fixtures.
- At least 30% reviewer-effort reduction at S1 without increased material escaped errors.

The S0 public corpus contains ten cases. Next gates are 50 fixture cases, at least 100 professionally reviewed cases, then at least 250 cases plus a hidden holdout. Hard integrity failures are reported separately, never averaged away.

## Claim ceiling and status

This package can test whether a mapping preserves or loses selected fields under a frozen synthetic contract. It cannot prove vendor scan correctness, actual export completeness, source authenticity, legal transfer permission, accounting correctness, filing compliance or product superiority.

The schemas and fixtures were fetched back from GitHub and parsed as JSON in a static authoring audit. Cross-references for relationship endpoints, purpose/data-element/access-principal refs, expected mapping losses, capability coverage, upstream origins and temporal exclusions were checked in a custom JS simulation. The Rust binary and its three tests are authored but **not executed**. No Cargo compile, cargo test, rustfmt, Clippy, Draft 2020-12 runtime pass, live API test, professional review or benchmark result is claimed.
