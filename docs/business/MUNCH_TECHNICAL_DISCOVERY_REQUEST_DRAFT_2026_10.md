# Munch Technical Discovery Request — Draft

**Purpose:** Explore a customer-authorised, read-only data integration for restaurant intelligence.  
**Status:** Draft for human review; not sent.

**Suggested recipient:** Munch Partnerships / Sales via https://www.munch.cloud/contact  
**Suggested subject:** Technical discovery — read-only restaurant analytics integration

---

Hello Munch team,

I’m exploring a vendor-neutral restaurant intelligence layer built to work alongside existing POS platforms rather than replace the checkout system. The goal is to help authorised restaurant operators evaluate ingredient demand, stock variance, waste-measurement quality and operational decisions using documented, reproducible evidence.

We are evaluating whether Munch can support a small, read-only pilot at one operator-authorised site. We would not request production credentials until scope, permissions, terms and the integration method are agreed.

Could you point us to the appropriate technical or partnership contact and clarify the following?

1. **Supported access path:** Is there a documented third-party API, partner programme, scheduled export or customer-authorised reporting interface for a read-only integration? If yes, could you share the current API documentation, version/deprecation policy, authentication model and required scopes?

2. **Available data:** Which objects can be accessed—sales transactions and line items, items/modifiers, discounts/tax, cancellations/voids/refunds, recipe/menu mappings, UOMs, stock movements/counts, purchase orders and Goods Receipt Vouchers? Are both detail rows and aggregate reports available?

3. **Event identity, delivery and correction semantics:** For event/webhook delivery, is there a stable source-issued event GUID or equivalent distinct from the order/line/object ID? Does that identity remain stable across redelivery and retries? Are attempt number, event type, event time and recorded/updated time exposed? Are messages signed, how should a consumer verify the signature against the exact raw body, and what are the documented retry/acknowledgement rules? How are edits, refunds, voids, cancellations, backdated receipts, reopenings, deletions and corrections represented—as separate events, versioned records, tombstones or replacement snapshots? Can an integration retain revision lineage without inferring updates from changed payloads?

4. **Extraction completeness and pagination:** For a declared site, query/filter and time interval, are consistent snapshots or export-session IDs available? What exact pagination contract applies to each relevant endpoint (opaque page token, offset/limit, page numbers or links)? How is terminal exhaustion signalled, can cursors repeat or expire, and what should a client do if source data changes during pagination or the server signals a restart/conflict? Are expected row counts, report-control totals, stable query identifiers, or a source-issued coverage statement available so that an operator can reconcile an export against the source?

5. **Evaluation environment:** Is a sandbox or anonymised example export available, ideally with expected row counts and a report/control total? We can start with a small sample and no customer personal data.

6. **Commercial and compliance requirements:** What customer consent, data-processing agreement, purpose limitation, retention/deletion, data residency, support/SLA, usage limits and integration fees would apply?

We are happy to start with the minimum supported interface and explicitly preserve limitations where the source cannot establish completeness.

For any fields or guarantees not supported, please mark them as unavailable or undocumented; we will treat them as unknown rather than synthesize provider-issued identity, authenticity or completeness. A small anonymised fixture that demonstrates duplicate delivery, a corrected/voided record, and the end-of-pagination behavior would be especially useful. We do not need customer names, phone numbers, card data, biometrics or employee-level attendance for the initial forecasting/ingredient-analysis question.

The first step is simply to understand what Munch supports, what is contractually permitted, and what evidence an authorised export can provide. We would not represent the integration as verified or make savings claims before the data and evaluation protocol are qualified.

Thank you,  
Luminous Dynamics

---

## Internal send checklist

- [ ] Confirm sender identity and public contact details.
- [ ] Decide whether to describe the pilot as a product/partnership enquiry or a prospective customer integration.
- [ ] Do not attach live POS records, credentials, customer data or employee data.
- [ ] Request written permission and current docs before implementing an adapter.
- [ ] Save the response and exact documentation/version references in the private commercial/funding workspace; keep the public repo limited to the generic, non-sensitive contract.
