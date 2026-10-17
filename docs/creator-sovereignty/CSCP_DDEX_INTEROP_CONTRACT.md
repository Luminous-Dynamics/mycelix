# CSCP DDEX Interoperability Contract (research draft)

**Status:** conceptual field-boundary contract; not a DDEX conformance claim and not an implementation guide for a production exchange.
**Related work:** Creator Sovereignty Control Plane RFC / draft PR #5166; asset passport core / draft PR #5182; tracker CSCP-INTEROP-001 (issue to be created).
**Research checked:** 2026-10-17.

## Executive decision

Do not create a competing music-industry interchange format. Keep CSCP's internal invariant model small and rights-aware, then put versioned adapters at the boundary to existing exchange standards. DDEX's Recording Data and Rights (RDR) family already separates repertoire/contributor/rights-claim notification, revenue reporting, rights-claim conflict messages, and message-exchange choreography:

- RDR-N 1.5: repertoire data, recording/video metadata, contributor identities and rights claims.
- RDR-R 1.1: revenue generated from use of recordings, music videos and performances, including revenue-report/summary/detail message shapes.
- RDR-RCC 1.0: specific rights-claim conflict communication when total claims for the same licensed right/resource exceed 100%, including notification and response messages.
- RDR-C 1.1: protocol/choreography for exchanging RDR messages; it is not the rights model itself.

Official references: [DDEX Recording Data and Rights](https://ddex.net/standards/recording-data-and-rights/), [RDR standards directory](https://kb.ddex.net/reference-material/standards-specifications/), [RDR-N implementation guidance](https://kb.ddex.net/implementing-each-standard/recording-data-and-rights-standards-%28rdr%29/recording-data-and-rights-notification-%28rdr-n%29/), [RDR-R guidance](https://kb.ddex.net/implementing-each-standard/recording-data-and-rights-standards-%28rdr%29/recording-data-and-rights-revenue-reporting-%28rdr-r%29/), [RDR-RCC guidance](https://kb.ddex.net/implementing-each-standard/recording-data-and-rights-standards-%28rdr%29/recording-data-and-rights-claim-conflict-%28rdr-rcc%29/), and [DDEX licensing guidance](https://kb.ddex.net/general-implementation-guidance/licensing-the-standards/).

DDEX states that its free Evaluation Licence permits evaluation and development/testing; an Implementation Licence is required before moving a standard implementation into production systems for commercial data exchange. Treat that as a release gate, not a footnote. Re-check the current licence and standard versions before implementation.

## Core paradigm shifts

1. **Internal truth model; external exchange contracts.** CSCP's type system exists to preserve distinctions and uncertainty. It is not a claim to replace DDEX's message schemas or industry role semantics.
2. **Import is evidence ingestion, not ownership adjudication.** A valid DDEX message proves that a payload fits a schema/version; it does not independently prove the sender's authority or the legal correctness of a rights claim.
3. **Revenue statements are not bank receipts.** RDR-R data describes reported revenue generated from use. Keep it as a sourced statement/claim with its reporting period, territory, currency, deductions and statement identity; reconcile actual cash separately.
4. **A conflict remains a conflict.** Preserve all source claims, status, dates and evidence. RDR-RCC is a communication flow for a specific rights-conflict use case, not a general court or a machine that resolves who owns the right.
5. **Unknown data is visible.** Never fabricate missing IDs, normalize incomplete shares, silently discard unsupported fields, or infer the work's ownership from an identifier.
6. **Lossless where possible; explicit loss otherwise.** Maintain raw source bytes (or a documented, access-controlled content-addressed reference), digest, schema version and parser/build identity. Sidecar every unsupported field or report that round-trip fidelity is not guaranteed.

## Conceptual mapping matrix

This matrix maps domain concepts, not exact XML element names. Exact field mappings and allowed value sets must be taken from the applicable licensed/current DDEX specification and its data dictionary. Do not implement from this table alone.

| CSCP internal concept | Candidate DDEX boundary | Adapter rule | Never infer |
|---|---|---|---|
| Musical work | Related work / identifier metadata supplied by source; ISWC may identify a work | Store the source-supplied identifier and namespace as an external ID; keep composition distinct from each recording | ISWC is not a recording ID, ownership statement or split agreement |
| Sound recording / music video | RDR-N resource metadata | Keep CSCP AssetId separate from ISRC and any DDEX resource/release identifiers; preserve source record ID | ISRC does not prove title, contributor consent or ownership |
| Performer / contributor credit | RDR-N contributor/party data; RIN may cover studio-session contribution roles and captured instrumentation/equipment | Store contribution claims/roles separately from rights claims and permissions; maintain exact source role vocabulary/version | A credit is not a confirmed royalty share or authorization to reuse audio/likeness |
| Recording-related rights claim | RDR-N rights-claim exchange | Preserve claimant, right type, territory, effective dates, basis, source, status and any source identifier as the schema permits | A syntactically valid or source-reported claim is not independently adjudicated legal truth |
| Claim totals / dispute lifecycle | RDR-RCC only when its defined >100% rights-claim conflict case and actor flow apply | Emit/consume notifications and responses only after validating the applicable profile, roles and required fields; keep dispute history append-only internally | Do not use RDR-RCC as a generic dispute API or auto-select an owner |
| Royalty/revenue statement | RDR-R 1.1, selecting the correct report/summary/detail message for the source use case | Import each statement/batch and line with source identity, period, territory, resource link, currency, gross/net/deductions and adjustments where actually provided | A reported revenue amount is not cash received; never silently combine currencies or infer missing deductions |
| Message transfer / partner exchange | RDR-C 1.1 when implementing RDR message choreography | Keep delivery, acknowledgement/retry, transport and partner identity separate from the payload semantics; retain transport receipts | Successful transport does not prove claim truth or payment settlement |
| Source export / parser trace | CSCP provenance sidecar (not asserted to be a DDEX field) | Bind raw payload digest, schema edition, parser version, import timestamp, source system, source batch/message ID, warnings and supported-field coverage | A content digest is not a signature, identity proof or lawful-consent proof |

## Recommended adapter architecture

1. **Quarantine input.** Store the original payload in creator-controlled, access-controlled storage; calculate a content digest; record source system, source record/message/batch ID, schema edition, parser version and import time. Keep original payload bytes immutable.
2. **Validate at the edge.** Validate against the exact schema and data dictionaries for the claimed/selected DDEX version. Reject unsupported major versions and malformed required fields. Validation failure must not mutate canonical passport state.
3. **Parse to source observations.** Convert only fields with a reviewed mapping. Each mapped field should carry source lineage or point back to an immutable payload/record locator. Unknown fields must be retained in the raw payload and reported in an import diagnostic.
4. **Map to the canonical core.** Preserve source IDs without replacing local AssetIds. Do not collapse work, recording, release, contributor, rights claim, usage grant, royalty statement and cash transaction into one “asset” record.
5. **Reconcile as proposals.** Conflicting imports create conflict states and review tasks. Never use “newest wins” or normalize a split to 100% unless a separately specified, evidenced rule allows it.
6. **Export by explicit profile.** Exports name the DDEX standard/version/profile and intended recipient/territory. An export is not marked conformant until it validates against the licensed, pinned schema and corresponding golden vectors.
7. **Retain a sidecar.** For each import/export, store a manifest of supported, transformed, omitted and unknown fields, plus raw payload digest and validation report. If full round-trip preservation is impossible, the UI/API must say so before export.

## Required data boundaries in CSCP

- Identifier record: scheme, exact original value, source record, observation time, schema/parser version, optional validation outcome. No generated IDs.
- Rights/contribution claim: claimant/party reference, target asset, rights pool/type, share where explicitly supplied, territory/time scope, state (proposed/confirmed/disputed/unknown), evidence reference and origin. Claim state must not silently become legal truth.
- Usage grant: asset, grantee, action, purpose, effective window, expiry/revocation event, issuer assertion, evidence reference. RDR notification does not by itself create the grant.
- Royalty statement: source statement/batch ID, reporting period, territory, resource mapping, raw amount/currency, deductions/adjustments if supplied, reported claimant/share if supplied, import provenance. Maintain a separate reconciliation link to a cash transaction only when actual cash evidence exists.
- Cash transaction: separate source account/transaction reference, posted date, currency and amount, with links to the statements it reconciles. A statement is not an actual transaction.
- Provenance sidecar: original bytes/digest, source/version, validation results, parser build and mapping version, field coverage, warnings and export decisions.

## Benchmark and release gates

The first adapter release should optimize for semantic integrity, not number of fields parsed.

| Gate | Required benchmark | Initial target |
|---|---|---:|
| Version handling | Wrong or unsupported schema edition is rejected with explicit diagnostic | 100% fixed fixtures |
| Schema validity | Every exported message validates against the chosen licensed schema and profile | 100% golden fixtures |
| Source fidelity | Import/export preserves every supported source field or reports the exact unsupported/changed fields | 100% of reviewed fixtures; zero silent losses |
| Identifier safety | No fabricated identifier; no work/recording/asset conflation | Zero violations |
| Claim preservation | Conflicting/proposed/disputed claims survive import/export without newest-wins resolution | 100% fixed fixtures |
| Rights-total conflict | A valid >100% conflict in the supported RDR-RCC use case is surfaced; no automatic rights adjudication | 100% applicable fixtures |
| Revenue-vs-cash boundary | A revenue statement never creates a cash receipt without separate transaction evidence | Zero false receipts |
| Currency / period / territory | No silent cross-currency aggregation; period and territory remain attached to every statement line where supplied | Zero boundary violations |
| Raw payload provenance | Every import/export has a payload digest, schema version, parser/mapping version and validation receipt | 100% records |
| Error atomicity | Failed parse/schema validation causes no canonical state mutation | Zero partial mutations |
| Permission boundary | Private catalogues/statements never leak to public indexes through an adapter default | Zero scope leaks |
| Production licence | Applicable DDEX Implementation Licence verified before commercial production exchange | Mandatory gate |

## Non-goals for v1

- Do not claim overall DDEX certification because a few fields map or an XML/TSV document parses.
- Do not send commercial messages to partners before licensing and recipient/profile agreement are complete.
- Do not implement a music royalty payout engine or payment custody.
- Do not resolve legal ownership or conflicts automatically.
- Do not make a streaming-platform API connector a prerequisite to support creator-provided exports.
- Do not assume DDEX's RDR family exhausts composition/publishing rights or all creator commerce; scope each adapter to its specified related-rights use case.

## Delivery sequence

1. Freeze the conceptual mapping and schema/version inventory; verify licensing requirements for evaluation vs production.
2. Collect synthetic, consented, or appropriately licensed fixtures for the exact schema version. Include malformed fields, unsupported versions, overclaim conflict, currency and territory boundaries.
3. Build one read-only RDR-N ingestion path with raw-payload provenance and no canonical mutation on validation failure.
4. Add RDR-R statement ingestion separately from bank/cash reconciliation.
5. Add RDR-RCC only for applicable conflict workflows, with human review and append-only history.
6. Add deterministic exports, conformance validation, source-fidelity diffs and independent fixture verification.
7. Consider live partner integration only after the Implementation Licence, security review, privacy/data-retention review and explicit operator/creator consent.

