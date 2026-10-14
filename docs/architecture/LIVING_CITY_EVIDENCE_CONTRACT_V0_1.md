# Living-City Evidence Contract (v0.1)

**Status:** proposed follow-on contract; not merged or independently validated by local execution.  
**Depends on:** Living-City concept record v0.1 in PR #5018.  
**Scope:** source metadata and bounded evidence claims for Sol-Atlas comparison and Mycelix provenance/governance.

## Design decisions

1. **Separate concepts from claims about the world.** A concept describes a hypothesis. An evidence record describes a specific claim and its relationship to a specific version of a concept hypothesis. A claim may support, challenge, contextualize, mix evidence, or remain unknown.
2. **Catalogue sources once; refer to them by stable IDs.** Source records include a canonical landing URL, publisher, source kind, publication/access date, declared version, spatial and temporal coverage, reuse status, and limitations. Evidence claims refer to source IDs instead of copying source metadata into every claim.
3. **Keep evidence class visible.** Distinguish observed measurements, published projections, administrative notices, catalogue discovery, community reports, reproducible analyses, simulations, assumptions, and contested interpretations. These types must not be silently promoted into one another.
4. **Represent uncertainty and missingness honestly.** Unknown resolution, dates, methods, scenario settings, and uncertainty bounds are explicit null/unknown values. No synthetic precision or inferred date ranges.
5. **Separate schema validation from semantic checks.** JSON Schema checks record shape, types, enums, and URI syntax. The CI-only semantic checker resolves concept/version/hypothesis and source references, enforces HTTPS-only links without embedded credentials, verifies date ordering, and applies selected conditional rules. It does not fetch remote URLs or certify source truth, safety, authority, or reuse rights.
6. **Keep the contract interoperable without claiming standards conformance.** Fields are informed by [W3C DCAT 3](https://www.w3.org/TR/vocab-dcat-3/) dataset/catalog concepts and versioning, and [W3C PROV-O](https://www.w3.org/TR/prov-o/) provenance relationships. [OGC STAC](https://www.ogc.org/standards/stac/) is a possible adapter for suitable geospatial assets, not a mandatory representation for every record. This JSON profile is not a complete DCAT, PROV-O, or STAC implementation.

## Record responsibilities

### Source record

The source schema describes one registry entry. The source fixture is a small source registry used by the first example set. The reuse state **open_license_verified** requires a license URL; it remains a recorded review assertion, not automated legal advice. Unknown licensing stays unknown.

Important fields include source identifier, title/publisher, kind, HTTPS landing URL, publication and retrieval dates, declared version, geographic and temporal coverage, separate administrative level and metric spatial resolution, an explicit resolution qualifier (exact/approximate/unknown/not applicable), reuse status, license URL if known, and limitations. The schema does not infer an API from a landing page or require external content to be copied into this repository.

### Evidence claim

The evidence schema describes a claim linked to concept ID, concept version, and hypothesis ID. It records evidence class, relationship to the hypothesis, source references, spatial/temporal applicability, method/protocol version, uncertainty, review status, limitations, and supersession links.

An **unreviewed** claim is not an approval. A **contested** claim remains visible as contested. A **withdrawn** claim should not be erased from governance history; downstream visibility must follow policy. A **reviewed** state requires a reviewer and decision timestamp at semantic validation.

## Johannesburg source inventory: six initial leads, not complete coverage

- **City of Johannesburg, Metroview announcement (15 September 2026):** [official announcement PDF](https://joburg.org.za/media_/Documents/2026-Media-Statements/Joburg-launches-Metroview-tool-during-Climate-Action-Plan-gathering.pdf). The sample claim records the announcement's reported future flood-exposure assessment as context only. It does not treat the announcement as the underlying dataset, verify its scenarios or spatial resolution, claim an API exists, or infer corridor-level flood conditions.
- **National Climate Change Information System (NCCIS), Climate Services catalogue:** [catalogue page](https://nccis.environment.gov.za/climate-services/download-data). The source record captures this page as a discovery lead. Each downstream dataset still needs its own version, scenario, temporal coverage, spatial resolution, methodology, access check, and license review.

- **South African Risk & Vulnerability Atlas (SARVA), South African Climate Risk Tool overview:** [SARVA portal](https://sarva.saeon.ac.za/). SARVA describes the tool as based on NASA NEX-GDDP-CMIP6 projections, bias-corrected and statistically downscaled to approximately 25 km, with information at local-municipality level. That is suitable as broad screening context, not as a street-level heat, flood, or intervention measurement. The underlying data/service version and reuse terms still need verification. The approximation qualifier is preserved so a map does not imply an exact 25,000 m resolution.

- **NCCIS Climate Change Projections:** [current page](https://nccis.environment.gov.za/climate-information/climate-projections). It describes national change maps comparing a 1971-2000 baseline with 2021-2050 projections, using the median of six dynamically downscaled GCMs under RCP 4.5 and RCP 8.5. It does not supply a single verified metric resolution for all map products. The baseline and future window remain distinct, and a specific download must be inspected before asserting its grid, version or license.
- **GreenBook MetroView:** [profile index](https://greenbook.co.za/tool-landing-page.html). The page states that selected metropolitan profiles provide quantified spatial information up to ward level on current and 2050 climate projections, hazard likelihood, exposure, vulnerability and demographic insights; it lists Johannesburg. Ward is an administrative level, not a uniform metre-resolution grid.
- **GreenBook Planning Support:** [reports, plans and maps/data packages](https://greenbook.co.za/greenbook-support.html). The page describes municipal data packages but the current visible listing does not conclusively establish the availability of a downloadable Johannesburg package. The PR records this as an open data-discovery question rather than claiming that the files are or are not available.

The accessed timestamp documents this research snapshot, not a guarantee that the pages remain unchanged. Future ingestion should capture source-version metadata and checksums for lawfully archived artifacts where practical.

## Fail-closed CI acceptance

The validator must:
- validate both schemas against Draft 2020-12 with an explicitly enabled FormatChecker;
- validate each source and evidence fixture, require unique IDs, and reject non-HTTPS links or URLs with embedded credentials;
- resolve every evidence source reference and every concept/version/hypothesis triple, and reject claims that assert a finer administrative level than their referenced sources support;
- reject reversed temporal ranges, unversioned simulations, observations lacking a named instrument-measurement method or temporal window, and reviewed states without reviewer/time;
- check license conditional rules, source-reported uncertainty bounds, self-supersession, temporal ordering, and consistency between a resolution value and its exact/approximate/unknown qualifier;
- prove selected protections with mutations for malformed URI format, unknown properties, unresolved source references, and unresolved hypothesis references;
- perform no external network calls and have no runtime side effects.

A successful run establishes structural and referential invariants for the checked fixtures only. It does not validate an entire city, certify source contents, establish permissions outside recorded review, prove community approval, or prove intervention effectiveness.

## Sol-Atlas display requirements

Show evidence class and review state beside every claim, link directly to source metadata, and expose spatial/temporal coverage and unknowns. Unknown must render as “not verified” or “not reported”, not zero or an empty map. A source with unknown rights may be linked as metadata but must not be redistributed or used as a basemap layer without rights review. Community reports need safe, consented publication practices; do not expose sensitive locations or identifiable contributors by default.

Keep minimum safety, accessibility, rights/consent, emergency access, and anti-displacement checks as gates. Show mobility, comfort, ecology, livelihoods, and lifecycle cost as separate indicator profiles with distributional impacts. No weighted aggregate score may compensate for an unmet hard gate.

## Next gates

1. Validate this stacked PR's exact-head workflow result; queued is not pass.
2. Review the field vocabulary and semantic rules before declaring this schema stable.
3. Define an auditable review, correction, and withdrawal protocol before importing third-party datasets or community-sensitive knowledge.
4. Build a read-only Sol-Atlas comparison view that labels projections, simulations, assumptions, community reports, and measured outcomes distinctly.
