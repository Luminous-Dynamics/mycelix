# Accounting evidence assertion matrix — smoke corpus v0.1

**Status:** research fixture interpretation; not an audit procedure, professional standard, assurance opinion or legal conclusion.  
**Parent:** FIN-ACC-BENCH-001 / #5782  
**Evidence model:** FIN-ACC-EVIDENCE-001 / #5798  
**Seed cases:** draft PR #5783  
**Reference verifier:** draft PR #5791

## 1. Why this matrix exists

The smoke corpus checks whether a small synthetic source bundle is internally consistent and whether its expected journal/close can be reconstructed. It does **not** establish that every document is authentic, the economic event was authorized, goods physically existed, title passed, the source universe is complete, or a bank transaction is finally settled.

The term `qualified_for_accounting_projection` in the seed data means only that the specified synthetic matching conditions are met under the declared fixture profile. It is not a real-world authorization decision or a representation that the source is sufficient for every accounting assertion.

Likewise, `qualified_settlement_observation` means a synthetic bank-statement-line observation matches the invoice identity/amount/entity fields. It does not independently prove payment-rail finality, beneficiary receipt, bank-origin authenticity or legal discharge of the debt.

## 2. Evidence dispositions are assertion- and purpose-specific

Use the following interpretation labels in this matrix:

- **Supports within fixture:** the provided bytes support a narrow declared fact inside the synthetic profile.
- **Contradicts:** the supplied bytes directly conflict with another supplied fact under the fixture rules.
- **Insufficient:** more evidence is needed to support the assertion.
- **Out of scope:** the seed contains no relevant evidence or policy contract for the assertion.

These labels are not a universal trust score. One source can support one assertion and be insufficient or irrelevant for another.

## 3. Source-to-assertion matrix

| Source | Narrow assertion it supports in this fixture | Assertions it does not establish alone | Key limitations |
|---|---|---|---|
| Purchase order (PO-001) | The synthetic entity's source bundle records an order for the stated SKU, quantity and price | Authorization by an appropriately delegated approver; supplier identity; receipt; liability recognition; payment; completeness | No signature, approval chain, supplier-master evidence or independent source authentication |
| Goods-received note (GRN-001) | The synthetic receiving record states that the declared quantity for the SKU was received | Physical existence/condition, independent warehouse confirmation, title/risks transfer, completeness of all receipts, acceptance by authorized staff | A document's existence does not prove the physical event; no independent inventory/warehouse evidence |
| Supplier invoice (INV-778 / INV-779) | The supplier-side record states the billed entity, counterparty, lines, quantities, unit price, total, date and currency | Supplier authenticity, legal enforceability, authorization, physical receipt, correct VAT treatment, accounting-policy appropriateness, complete liabilities | Issuer authenticity and tax evidence are not included; no contract, tax registration or source-channel proof |
| Bank-statement line (BANK-LINE-001) | The fixture records a debit observation with a reference and amount that match the invoice | Authentic bank origin, final settlement, beneficiary receipt, irrevocability, payment authorization, legal discharge, absence of reversal/chargeback | No signed bank feed, independent bank confirmation, account ownership proof, payment status lifecycle or return/reversal search |

## 4. Case-by-case assertion coverage

### ACC-SYN-0001 — PO, GRN, invoice and bank debit observation

**Supported within the synthetic profile:**
- The PO, GRN and invoice declare the same entity, counterparty, currency, SKU and quantity.
- PO unit price and line total equal the invoice values.
- Quantity × unit price equals the declared line total using exact integer arithmetic.
- The bank-line observation references the invoice and matches the declared full amount and entity/counterparty/currency.
- The two expected journal entries are balanced and the independent close calculation can be reconstructed from the fixture's opening balance and expected journal oracle.

**Insufficient or out of scope:**
- PO authorization and segregation of duties.
- Supplier/bank account authenticity and changed-bank-details fraud.
- Independent proof of physical goods receipt and condition.
- Whether inventory recognition is correct under an approved real accounting framework.
- Whether a payable was legally extinguished by payment.
- Whether a bank debit represents final settlement or could be reversed.
- VAT/tax treatment, disclosures, FX or statutory framework compliance.
- Evidence-population completeness for the entity/period.

### ACC-SYN-0002 — exact duplicate invoice ingestion

**Supported within the synthetic profile:**
- Repeated document ID plus identical content fingerprint is treated as an idempotent ingestion no-op.
- Only the single expected invoice journal appears in the oracle.

**Insufficient or out of scope:**
- Semantic duplicate detection where the same economic invoice is reissued with a new ID or modified bytes.
- Detecting collusive supplier/employee activity.
- Proving the original invoice is authentic or authorized.
- Proving no other copy exists in another channel/system.

An exact digest duplicate is a narrow identity/replay case, not a complete duplicate-invoice fraud solution.

### ACC-SYN-0003 — invoice addressed to the wrong legal entity

**Supported within the synthetic profile:**
- The invoice's declared entity differs from the PO/GRN entity.
- The specified profile's disposition is quarantine with no journal entry.

**Insufficient or out of scope:**
- Resolving a real-world legal entity from weak identifiers.
- Determining whether an intercompany charge, agent, branch, central purchasing arrangement or legal novation makes the invoice valid.
- Confirming the supplier and recipient identities independently.
- Applying consolidation, transfer-pricing, VAT-group or branch accounting rules.

A conflict should remain quarantined/indeterminate until the appropriate entity and contractual evidence is supplied; do not hard-code every differing identifier as a universal legal conclusion.

## 5. Assertion coverage dimensions for expansion

Every future benchmark case should declare the required assertion/procedure pairs. At minimum:

| Assertion | Example evidence obligation | Negative control |
|---|---|---|
| Existence / occurrence | External or independent source supports that the recorded item/event occurred | Balanced journal for an invoice with no receipt or service evidence |
| Completeness | Expected population and control total are independently enumerated | Omitted bank account, AP feed, payroll run or subsidiary |
| Rights and obligations | Contract/ownership/beneficiary/obligation evidence binds the entity to the item | Invoice billed to another legal entity; disputed ownership/title |
| Accuracy / valuation | Exact quantities, price, FX, allocation, tax and valuation inputs are supported | One-minor-unit difference; stale FX; incorrect unit; unsupported estimate |
| Cutoff | Event time, delivery/service period, invoice date and posting period are qualified | Late invoice, backdated approval or receipt after cutoff |
| Classification | Approved profile/rule and relevant facts support the selected account/category | Correct arithmetic posted to the wrong asset/expense/liability class |
| Presentation / disclosure | Required roll-up, classification, disclosure and taxonomy version are supported | Omitted related-party disclosure or stale XBRL taxonomy |
| Authorization | Principal, delegation, limits, approvals and separation of duties are verified | Unauthorized but balanced posting |
| Settlement/finality | Authoritative payment/clearing observation, status lifecycle and reversals are checked | Provider acknowledgement or bank debit treated as final settlement |

## 6. Evidence independence rules

1. Two copies from the same upstream source are one evidence origin, not two independent corroborations.
2. A management report generated from the same GL is not independent evidence of the GL's completeness.
3. A supplier statement and invoice may be separate documents but not independent evidence if both originate from the same supplier system.
4. A source hash proves that the captured bytes have not changed relative to that digest; it does not prove source authorship or integrity before capture.
5. A source feed's absence is not evidence of zero activity. Zero requires affirmative evidence and an appropriately qualified source population.
6. A model explanation or citation string does not establish that the cited source entails the assertion.

## 7. Required future result contract

For each material assertion and intended procedure, return:
- `supported_within_declared_scope`;
- `contradicted`;
- `insufficient_evidence`;
- `indeterminate`;
- `not_applicable_with_rationale`; or
- `needs_authorized_review`.

Bind the result to the exact entity, period, policy/knowledge frontier, evidence scope manifest, source refs, evidence-origin groups, evaluator revision and known limitations. Preserve positive and negative evidence; never collapse them into a single confidence field.

## 8. Review gates before stronger claims

- Obtain accountant/auditor review of assertion mappings and evidence-purpose semantics.
- Add signed/authenticated source-channel evidence before testing authenticity.
- Add independently confirmed receipt and bank settlement lifecycles before claiming occurrence or settlement assurance.
- Add source-universe qualification from #5790 before claiming completeness within scope.
- Add approved temporal policy snapshots from #5792 before claiming policy correctness.
- Add evaluator-isolated candidate/human arms from #5799 before reporting performance comparisons.

**Claim ceiling:** this matrix clarifies only what the three synthetic smoke fixtures do and do not support. It does not establish real-world truth, complete books, audit sufficiency, compliance or assurance.
