# Mycelix Music: Industry Comparison and Evidence-Led Roadmap

**Status:** research and product-planning document  
**Research checked:** 13 October 2026  
**Scope:** music distribution, rights metadata, royalty reporting, settlement accounting, creator experience, privacy and operational maturity.

## Executive assessment

Mycelix Music's strongest defensible differentiator is not a promised payout multiple. It is the design direction toward a verifiable creator-accounting chain: source evidence → obligations → deductions/adjustments → netting → rail receipts → allocation lineage → statement, with explicit completeness and provenance boundaries.

That is a promising architecture, not yet proof of a superior production service. Mature music-industry operators have catalogue scale, established licensing relationships, distribution operations, payment pipelines and standards-based data exchanges. Mycelix must demonstrate equivalent reliability and interoperability while making its audit trail and creator-controlled economics useful to actual artists.

**Current comparison rule:** distinguish (1) industry fact, (2) Mycelix code or design currently visible in the repository, (3) a planned capability, and (4) a measured outcome. Do not present a target as an outcome or infer market-wide behavior from one company.

## What the industry actually does

### Streaming royalties are not a universal per-stream rate

Spotify's own explanation says recording and publishing royalties are allocated from roughly two-thirds of its music revenue and distributed according to streamshare; the rights holders then pay artists and songwriters according to their individual agreements. It explicitly says there is no fixed per-stream rate. Spotify also reported more than US$11 billion paid to rights holders in 2025. These are Spotify's own figures and model, not a universal benchmark for all services.

Sources:
- [Spotify for Artists — Royalties Guide](https://artists.spotify.com/en/royalties-guide)
- [Spotify Loud & Clear — per-stream-rate explanation](https://loudandclear.byspotify.com/faqs/i-heard-spotify-pays-a-fraction-of-a-penny-per-stream-is-that-true/)

**Implication for Mycelix:** a pay-per-play or creator-guarantee strategy can be a deliberate alternative, but it must state who funds it, which revenue pool it allocates, how splits and deductions work, what happens on refunds or fraud, and whether the stated amount is gross, net, accrued, or actually paid. A single dollar figure per stream cannot fairly compare different business models.

### Interoperability is an industry capability, not an optional polish item

DDEX publishes multiple distinct standards for different jobs in the value chain. Current published versions include:
- **ERN 4.3.2** for release/resource metadata and availability/deal information delivered to digital services.
- **RDR-R 1.1** for revenue reporting relating to use of sound recordings, music videos and the performances they contain.
- **RIN 2.1** for contributors, roles, creations and studio-session information.
- Other DDEX standards address recording rights/claims, digital sales and usage reporting, work registration and metadata enrichment.

Sources:
- [DDEX current standards specifications](https://kb.ddex.net/reference-material/standards-specifications/)
- [ERN overview](https://ddex.net/standards/electronic-release-notification-message-suite/)
- [RDR overview](https://ddex.net/standards/recording-data-and-rights/)
- [RDR-R version 1.1](https://kb.ddex.net/implementing-each-standard/recording-data-and-rights-standards-%28rdr%29/recording-data-and-rights-revenue-reporting-%28rdr-r%29/)
- [RIN overview](https://ddex.net/standards/recording-information-notification/)

CISAC's **Common Royalty Distribution (CRD)** format is designed for reporting distributed royalties between societies and members. In June 2026, CISAC also launched **AVR+**, a machine-readable JSON format based on its global cue-sheet standard for audiovisual music usage.

Sources:
- [CISAC — CRD and other electronic exchange formats](https://www.cisac.org/formats)
- [CISAC — AVR+ announcement, 18 June 2026](https://www.cisac.org/Newsroom/news-releases/cisac-launches-first-global-format-modernise-audiovisual-music-data-and)

DDEX says implementers can evaluate and test standards under its evaluation licence, but need its implementation licence before exchanging standard-formatted data in production. Confirm the applicable licence and terms before enabling any production connector.

Source: [DDEX — licensing the standards](https://kb.ddex.net/general-implementation-guidance/licensing-the-standards/)

For US musical-work mechanical royalties, the Music Modernization Act established a blanket-licensing framework and the Mechanical Licensing Collective's collection, matching and distribution responsibilities. It illustrates that catalogue matching, ownership data and exception handling are core operating requirements—not just export formats.

Source: [US Copyright Office — Music Modernization Act](https://www.copyright.gov/music-modernization/)

## Comparative scorecard

| Dimension | Established industry baseline | Mycelix Music position to substantiate | What would demonstrate real progress |
|---|---|---|---|
| Catalogue and reach | Large services and distributors have established catalogues, ingestion pipelines and audience relationships. | The repository contains platform, API, contract and playback work; code presence alone does not establish catalogue rights, DSP distribution or audience scale. | Rights-cleared pilot catalogue, successful ingestion, playback reliability and repeat listening. |
| Creator economics | Models differ: subscriptions/advertising, direct sales, patronage, licensing and distributor/label agreements have different fee bases and risk. There is no universal per-stream rate. | Multiple economic strategies and a more explicit gross-to-net accounting model are promising design assets. A payout multiple is not demonstrated by source code. | Publish real, opt-in pilot receipts with gross revenue, each deduction/fee, net creator allocation, settled amount and funding source. Compare like-for-like models. |
| Royalty accounting | DDEX, CISAC formats, collective-management workflows and contractual statements support exchange and distribution processes; maturity varies by participant. | Mycelix's accounting package models obligations, deductions, settlement evidence, statement projections, commitments and lineage. The new checkpoint work remains under qualification. | Exact-head tests pass; reconciliation against a real partner statement; independently reviewed invariants; exception and correction process. |
| Interoperability | Standards define specific messages, versions, allowed values and exchange workflows. | The code's DDEX/CRD adapters are minimal **mapping projections**, not complete official DDEX XML/flat-file or CISAC CRD implementations. | Version-pinned schema adapters, official fixtures/schema validation, identifier mapping, licensed production exchange and partner acceptance. |
| Auditability | Audit and statement detail vary across systems; do not assume every incumbent has or lacks any particular proof facility. | Ordered commitments and checkpointed lineage can make a precise history easier to reconstruct and verify if source assertions, keys, snapshots and stored evidence are trustworthy. | Publish independent verifier results, a redacted sample proof packet, rollback/tampering tests, correction/appeal semantics and measured reconstruction time. |
| Privacy | Services and territories differ in their data collection, disclosure, retention and legal obligations. | Privacy and selective disclosure are goals; a Merkle commitment or inclusion proof is not by itself a zero-knowledge proof or a privacy guarantee. | Data-flow inventory, threat model, data minimization/retention rules, disclosure tests and a separately reviewed ZK design before claiming ZK privacy. |
| Settlement | Timing depends on the licensing relationship, reporting cycle, minimum thresholds, banking/payment rails, disputes and compliance checks. No blanket 60–90-day claim covers all arrangements. | Faster accounting is an architectural goal; it is not the same as cash being final and available to the creator. | Report P50/P95 from usage receipt to statement, accrual to authorization, and authorization to final cash settlement—separately. Include reversals, holds, failed payments and all-in fees. |
| Governance and resilience | Large platforms operate under centralized commercial, legal and technical controls; decentralization is not binary. | Federated/Holochain/community-governance designs can reduce certain dependencies, but do not make a service “unstoppable” or immune to legal, key, hosting or governance failures. | Demonstrate node diversity, recoverability, dispute/moderation policy, key rotation, upgrade governance, content portability and service continuity under node loss. |
| Operational maturity | Established providers have ongoing operating teams, partner integrations, customer support and production incident history. | This repository is open-source development, not evidence by itself of production scale, independent security audit, payment licensing or partner adoption. | Named pilot, user-approved metrics, incident/runbook exercises, independent security review and repeatable release qualification. |

## Claims policy for the public roadmap

Until supported by measured evidence, describe the following as **targets or hypotheses**, not present facts:

- “10x” or “10–50x” higher artist payouts.
- A universal “$0.01 per stream” guarantee or a universal competitor rate.
- “1% platform fee” without listing payment processing, gas, storage/CDN, conversion, fraud, refund and operating costs.
- “Instant settlement” without defining finality and measuring the end-to-end path to usable funds.
- “Zero-knowledge privacy” when the implementation is a hash/Merkle commitment or selective-disclosure scheme without a verified zero-knowledge proof system.
- “Unstoppable,” “no competitor,” or other exclusivity claims without a bounded, independently reproducible comparison.

This does not diminish the vision. It makes each claim falsifiable and helps artists understand the offer before they trust it with rights or money.

## Recommended roadmap: prove one complete vertical slice first

### P0 — Qualification and truthful evidence
1. Complete exact-head TypeScript and PostgreSQL qualification for accounting/checkpoint changes. Queued or stale workflows are not passes.
2. Freeze a canonical statement fixture containing usage, rights resolution, obligation, deduction, settlement receipt, allocation lineage and statement output.
3. Independently reconstruct the statement from persisted evidence and compare every amount, root and completeness boundary.
4. Publish a sanitized evidence packet: inputs and versions, source watermarks, statement root, verifier result, known limits and correction procedure.

### P1 — Standards-aware identifiers and mapping
1. Define internal canonical IDs and explicit mappings for works, recordings, releases, parties, rights shares, territories, currencies, usage periods and settlement references.
2. Preserve source IDs and source rows; never infer a rightsholder identity from a fuzzy match without recording confidence, competing candidates and a human-review path.
3. Do not relabel a Mycelix-native accounting unit as ISO 4217 merely because it is a non-empty string. Require an explicit supported currency mapping; any conversion needs an authoritative, dated FX quote, disclosed rate, rounding policy and separately committed evidence.
4. Implement the smallest useful connector first—likely a version-pinned royalty/revenue import/export path—and keep each incoming report immutable with a digest and ingestion receipt.
4. Validate actual standard messages against current DDEX/CISAC schemas and sample files. Label a mapping object as non-conformant until this passes; acquire the DDEX production implementation licence before production exchanges.

### P2 — Artist-visible transparency
For every statement line, show the source, work/recording reference, reporting period, gross basis, split/share, deductions with authority, held balance and reason, net payable, paid amount and rail receipt. Distinguish **accrued**, **eligible**, **authorized**, **submitted**, **finalized**, **reversed** and **disputed**.

Make corrections append-only: a corrected statement should reference its predecessor, explain the delta and preserve both versions. Let an artist export the statement and verification bundle without needing a Mycelix account.

### P3 — Pilot metrics before payout claims
Start with opt-in, rights-cleared artists and a bounded catalogue. Publish a methodology before targets:
- ingestion coverage and unmatched-usage rate;
- match precision/recall on a labelled sample, with the human-review burden;
- statement correction and dispute rate;
- amount reconciled exactly between source report and statement;
- median and 95th-percentile time for each settlement stage;
- all-in cost and net creator amount by economic model;
- system availability, recovery time and failed/reversed payment rate.

Report sample size, date range, exclusions and limitations. Separate synthetic fixtures from real production observations.

### P4 — Expand only after the evidence holds
Then add additional territories, audiovisual cue sheets/AVR+, contributor studio metadata/RIN, catalogue/right-share exchanges, additional payment rails and federated operators. Make each integration optional and replaceable; do not make the core ledger dependent on a single distributor, standards provider, chain or service operator.

## Decision

The credible competitive position is **creator-controlled economic choices plus a verifiable accounting and correction trail**. The industry gap to close is production execution: standards-conformant connectors, source-quality and matching workflows, independent qualification, real payout receipts, support and adoption. Do those first, then publish comparisons against measured baseline data rather than advertised multiples.

*Sources and standards versions checked 13 October 2026. Verify the relevant official schema/version, implementation licence and partner profile again before any production integration.*
