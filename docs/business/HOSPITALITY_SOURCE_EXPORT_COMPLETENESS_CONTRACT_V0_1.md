# Hospitality Source Export Completeness Contract v0.1

Status: **proposed evidence contract; read-only and non-authorizing**

## 1. Purpose

The hospitality qualification work already preserves the exact export bytes, import diagnostics, transaction-derived forecast actuals, separate control documents, and external authenticity evidence. It still must not infer that the provider exported every qualifying event merely because an export parses or reconciles to a locally calculated total.

This contract adds a separate, explicitly bounded question:

> What evidence shows that the declared extraction procedure traversed the entire source query for the exact business scope and half-open time window?

It is intended for restaurant POS/order exports and, with suitable mappings, related inventory and supplier-event exports. It is not a vendor-specific Munch adapter and does not assert that any vendor exposes the necessary API or attestation today.

## 2. Four claims that must not be collapsed

| Claim | Evidence needed | What it does not establish |
|---|---|---|
| **Transport/pagination traversal** | Exact query identity, stable snapshot identity, ordered page receipts, continuous opaque-cursor hand-offs, explicit terminal page, page payload digests and row counts | That the query includes every logically relevant event |
| **Provider-declared query coverage** | Provider statement binding source, scope, query/filter digest, schema, snapshot, [start, end) window, declared count, and statement digest | That the provider statement is authentic or its semantics are authoritative |
| **Authenticated provider statement** | External verifier receipt bound to the exact statement digest, claimed issuer, verifier policy, credential/revocation context, and validity interval | That the issuer is authorised to define the economic metric or that the POS recorded every real-world event |
| **Independent reconciliation / semantic authority** | Separately issued control totals and coverage controls, plus explicit authority for the metric definition and event-status semantics | Unbounded claims about physical reality beyond the independent controls' scope |

A pagination chain can be complete while the provider query omits refunds, voids, late-arriving events, a second location, or a relevant status. A correctly authenticated statement can faithfully attest to a flawed query. A matching aggregate can conceal offsetting missing and extra events. These are distinct failure modes.

## 3. Exact extraction binding

A source export manifest should bind, at minimum:

- source-system identity and restaurant/business scope;
- exact query and filter digest, including event-time field, event statuses, order types, and any exclusion rules;
- source schema/version digest, versioned source-contract semantics digest, and local mapping digest;
- a source snapshot or export-session identity;
- one half-open interval `[start, end)`, represented in UTC with the separately preregistered local-time interpretation;
- page ordinal, request cursor digest, returned next-cursor digest, page payload digest, and returned-row count for every page;
- the explicit terminal-page observation; and
- a provider coverage statement that names the same scope, query, snapshot, schema, and interval.

The cursor values are opaque. Mycelix checks the chain for continuity, cycles, and terminal state; it does not interpret the token or assume that a vendor's cursor semantics are sound.

The v0.1 Rust contract in `crates/mycelix-business-export-completeness` performs structural checks only. It requires page ordinals to be contiguous from zero, the first request to be explicitly initial, every later request cursor to match the previous page's next cursor, exactly the final page to declare `Terminal`, shared snapshot/query/schema/source-contract/window identity, overflow-safe row counting, and agreement with the provider-declared row count. Missing cursor evidence is distinct from explicit terminal evidence. It also requires the provider's stated coverage claim to be `CompleteForDeclaredQuery`. The result carries domain-separated SHA-256 digests for the ordered manifest, parsed coverage projection, and combined evidence. The additive `assess_export_completeness_with_raw_payloads` entry point additionally hashes each supplied raw page and the raw coverage statement and requires the computed SHA-256 values to equal the manifest and attestation digests. This makes the byte identity of supplied artifacts checkable, but it does not authenticate the provider, derive the semantic projection from those bytes, or certify source query semantics.

## 4. Source statement and trust boundary

The provider statement is a claim. A caller-provided digest is an identifier for an artifact, not proof that the artifact was signed or issued by the named system. Likewise, the local parsed-projection digest only makes the interpreted fields reproducible; it does not prove those fields came from the exact statement bytes. A higher verifier must authenticate the raw statement and independently bind its parse to this projection.

Before treating a provider statement as authenticated, a higher evidence layer must verify an externally issued receipt against the exact statement digest and claimed issuer. That verifier must apply a pinned policy and revalidate relevant credential, verifier, validity, and revocation context at the qualification point. A changed trust frontier requires fresh evaluation; it must not be silently interpreted as continued validity.

Even an authenticated statement needs explicit source-contract semantics. The source specification must say what the query means, whether it returns a consistent snapshot, which event classes and statuses are included, how corrections are represented, how far late-arriving events can revise a closed interval, and whether the stated terminal cursor really means exhaustion of the declared query.

The v0.1 assessment never removes the following limitations by itself:

- source-statement authenticity unverified;
- source query/metric semantics not independently authorised;
- physical-world completeness unverified; and
- independent control reconciliation not established.

The type returned by the crate is intentionally named `StructurallyConsistentExport`, not `VerifiedCompleteExport`.

## 5. CSV/manual export policy

A successfully parsed CSV file proves that the observed bytes were parsed under one adapter and mapping. It does not prove that the POS exported every qualifying transaction.

A manual export without source snapshot, page-chain, or equivalent source-issued coverage evidence remains **completeness unknown**. Operators must not manufacture cursor continuity from file rows, extrapolate a final page from the absence of a next-page file, or substitute file row count for a source-issued expected count.

Manual export pilots can still proceed at the evidence level they actually support. Preserve the raw export and its digest, keep the full discovered-row denominator and all rejection classes, reconcile against independent window controls when available, and carry the upstream-completeness limitation when the necessary source evidence is absent.

## 6. Event semantics that must be frozen

For POS sales and order events, the connector contract must declare:

- order-created time versus accepted/paid/fulfilled time used for inclusion;
- whether cancelled, voided, refunded, partially refunded, reopened, or corrected transactions appear as events, state snapshots, or adjustments;
- whether an event can be updated after initial export and how those revisions are identified;
- treatment of tips, discounts, delivery fees, taxes, service charges, and currency precision;
- location, register, channel, and order-type scope;
- source timezone versus event timezone, and the exact half-open interval boundary;
- stable source-event identity and duplicate/replay behavior; and
- how late arrivals and closed-period restatements are surfaced.

The digest of these semantics must be frozen before the evaluation window. A semantic change creates a new mapping/qualification lineage; it must not rewrite a prior assessment.

## 7. Food traceability and interoperability

Use established supply-chain standards where the source event domain fits them. GS1 EPCIS 2.0 provides common event semantics and REST bindings for supply-chain visibility, including product movement/status and traceability across trading partners: <https://www.gs1.org/standards/epcis> and <https://ref.gs1.org/standards/epcis/2.0.0/>.

EPCIS is a useful candidate for supplier dispatch, receipt, lot, location, and custody events when participants can emit standards-conformant data. It is not a reason to force ordinary POS orders into an EPCIS model. The connector must preserve the source's original event identity and map to a normalized contract without erasing source-specific semantics.

## 8. Adversarial cases required before a stronger qualification

The contract's tests and future provider adapters should reject or quarantine:

1. Missing middle page or skipped page ordinal.
2. Repeated/cyclic cursor or a cursor hand-off mismatch.
3. A final page that still advertises a next cursor, or a premature terminal page.
4. Snapshot/query/schema/window drift between pages.
5. Attestation scope, interval, schema, query, snapshot, or row-count mismatch.
6. A partial/unknown provider claim presented as complete.
7. Zero or malformed payload/statement digests.
8. Arithmetic overflow in observed row-count aggregation.
9. Export issued or attested before its declared interval is closed.
10. A valid cursor chain over a query with undeclared status filters or incomplete event semantics.
11. Authenticated but unauthorised metric semantics.
12. A provider control total that reconciles despite offsetting missing and extra events.

The last four require source-policy, independent-control, or semantic-authority evidence beyond this crate's structural verifier. They must not be falsely described as covered by unit tests for cursor continuity.

## 9. Proposed evidence states

Keep the states monotonic only when new evidence is added; a stronger label must not overwrite the original artifacts.

- **Unknown** — no source-level completeness evidence.
- **StructurallyConsistent** — page chain and supplied provider claim match the contract.
- **AuthenticatedStatement** — an independent verifier authenticates the exact statement under a pinned policy.
- **ReconciledCoverage** — independently issued controls cover the registered interval without gaps/overlaps and reconcile under the declared aggregation.
- **SourceSemanticsBound** — a versioned source contract establishes the meaning and coverage semantics of the exact query.
- **Unresolved** — required artifacts are missing, mismatched, stale, revoked, or outside their validity interval.

These labels describe dimensions of evidence, not a universal Boolean called "complete." The evidence bundle must retain the exact scope/window and remaining limitations. Completeness for one location, query, status set, and interval must not be projected onto another.

## 10. Research connection and evaluation guardrails

Food demand forecasting research reinforces that the quality of a model must be tested under temporally valid splits and compared with suitable baselines; it does not turn synthetic targets or an incomplete source export into ground truth. For example, a 2025 food-demand study evaluates an ensemble against simpler models and explicitly discusses freshness variables and waste impact: <https://www.sciencedirect.com/science/article/pii/S2772390925000241>. Recent intermittent-demand research also emphasises time-respecting evaluation and leakage avoidance: <https://www.nature.com/articles/s41598-026-35197-y>.

For this product, keep the following conclusions separate:

- forecast error against supplied export-derived actuals;
- extraction and source-coverage evidence;
- measured waste/stockout outcomes;
- causal operational impact; and
- business execution authority.

A model can perform well against incomplete labels and still be operationally wrong. No percentage-savings claim follows from structural export completeness or model qualification alone.

## 11. Acceptance boundary

This contract is accepted only as an additive, non-authorizing evidence layer. It must not:

- write to a POS, inventory, purchasing, payment, payroll, or scheduling system;
- mint authority, reservations, or execution receipts;
- remove limitations based solely on local file counts;
- treat a provider statement as self-authenticating;
- claim universal source completeness; or
- claim any measured improvement over Munch or another product.

The next evidence-producing step is a real adapter for one source system with documented query semantics and genuine source-issued coverage evidence. Until then, the structural verifier can be unit-tested, but field completeness and commercial value remain unproven.
