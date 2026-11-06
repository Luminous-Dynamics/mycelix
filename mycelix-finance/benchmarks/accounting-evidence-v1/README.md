# Accounting Evidence Benchmark — smoke corpus v0.1

This directory contains a tiny synthetic smoke corpus for the protocol in [ACCOUNTING_EVIDENCE_BENCHMARK_V1.md](../../docs/ACCOUNTING_EVIDENCE_BENCHMARK_V1.md).

## Included cases

1. ACC-SYN-0001: positive control for purchase-order, goods-received, invoice and bank-settlement evidence, with exact journal and close oracle.
2. ACC-SYN-0002: exact duplicate invoice ingestion must be a no-op.
3. ACC-SYN-0003: wrong legal-entity invoice must be quarantined even when supplier, amount, PO and delivery evidence otherwise align.

## Monetary and accounting scope

- All monetary amounts are canonical non-negative integer strings in minor units; currency and minor-unit scale are explicit.
- The fixture uses a synthetic ZAR 2-decimal profile for numerical examples only.
- Tax is explicitly excluded. Zero tax in this corpus is not a statement about South African VAT or any real tax treatment.
- The policy identifier is synthetic and is not IFRS, IFRS for SMEs, GAAP, SARS guidance, statutory advice or an approved production policy.
- The opening trial balance is an oracle input, not an inferred fact.
- Each source document fingerprint is SHA-256 over canonical UTF-8 JSON of the nested content object, serialized with sorted keys, no insignificant whitespace and UTF-8 characters preserved.

## Claim ceiling

These three cases only smoke-test schema shape, source binding, idempotency, entity boundaries, balanced journal arithmetic and close reconstruction. They are not a benchmark result, an accuracy estimate, evidence of accounting competence, or production readiness.

## Next corpus expansion

Add independent tests for partial settlement, credit notes, altered source fingerprints, missing goods receipt, FX mismatch, cutoff errors, policy revision, source prompt-injection, hidden liabilities, intercompany elimination and statement reconstruction. Preserve the public/hidden split boundary before training or optimization uses any case.
