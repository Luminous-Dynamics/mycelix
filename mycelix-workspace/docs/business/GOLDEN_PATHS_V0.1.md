# Golden Paths v0.1

Status: **normative candidate / documentation only**

These six workflows are the semantic qualification corpus for the Business institutional substrate.

The purpose is not to specify UI screens. The purpose is to prove that one small ontology can express materially different business activity without letting Business orchestration become an authority oracle.

Each path is written as:

1. commitments/obligations created;
2. authority required;
3. domain-owned actions/outcomes;
4. external observations where applicable;
5. reconciliation;
6. closure predicate;
7. compensation/dispute behavior.

## GP-001 — Product sale

### Intent

Sell a physical product to a customer for an agreed price.

### Normative shape

`CommercialProposal`

-> `CommercialAgreement`

-> `PaymentObligation(customer -> business)`

+ `FulfillmentObligation(business -> customer)`

### Domain ownership

- Party identity: Identity.
- Offering/order/agreement: Commerce.
- payment authority/settlement: Finance.
- stock/custody/fulfillment: Supply Chain.
- accounting recognition: future Accounting kernel.

### Execution shape

1. Commerce validates the offering and creates/accepts the order/agreement.
2. Finance establishes or references the payment obligation and requests payment through an authorized rail/provider.
3. Provider responses are retained as observations, not final truth.
4. Supply Chain reserves/releases authoritative inventory according to its own rules.
5. Fulfillment is observed and accepted by Supply Chain.
6. Finance reconciles accepted settlement observations to the exact payment obligation.
7. Accepted economic events are projected to Accounting.
8. Receipt/document projection may be generated after its required predicates are met.

### Closure predicate

The sale MAY be `Closed` only when the exact closure policy establishes, at minimum:

- commercial agreement remains valid for the transaction;
- payment obligation is in an accepted terminal state appropriate to the sale policy;
- fulfillment obligation is satisfied or explicitly waived/compensated;
- required inventory effects are accepted by Supply Chain;
- required accounting events are accepted by Accounting;
- unresolved disputes/unknown effects do not contradict closure.

### Adversarial vectors

- provider says paid, bank reconciliation absent;
- settlement succeeds, inventory reservation fails;
- fulfillment succeeds, settlement later reverses;
- duplicate payment callback;
- duplicate workflow replay;
- customer disputes delivery;
- stale approval authority;
- stock state unavailable.

The workflow MUST remain partial/disputed/compensation-required rather than inventing complete success.

---

## GP-002 — Service sale

### Intent

Provide scoped work to a customer and collect payment.

### Normative shape

`ServiceProposal`

-> `ServiceAgreement`

-> `WorkObligation(business -> customer)`

+ `PaymentObligation(customer -> business)`

### Domain ownership

- parties/credentials: Identity;
- commercial agreement/invoice: Commerce;
- work/project outcome: owning service/work domain;
- payment/reconciliation: Finance;
- books: future Accounting kernel.

### Execution shape

1. Commerce records the accepted service agreement.
2. Agreement instantiates work and payment commitments.
3. Work domain records authoritative work evidence/outcomes.
4. Commerce produces invoice/document projection from the payment obligation and service context.
5. Finance receives external settlement observations and reconciles them to the exact obligation.
6. Accounting consumes accepted service/performance/payment economic events under accounting policy.

### Closure predicate

The engagement MAY close when:

- required work obligations are satisfied, waived, or explicitly terminated;
- payment obligations are reconciled or explicitly written off/waived under valid authority;
- required documents/accounting events are accepted;
- no unresolved dispute prevents the selected closure class.

### Adversarial vectors

- customer pays before work is accepted;
- work is accepted but payment fails;
- invoice corrected after issue;
- time/work evidence is disputed;
- payment duplicated;
- agreement is terminated mid-project;
- responsible approver loses authority mid-workflow.

---

## GP-003 — Procurement

### Intent

Purchase goods or services from a supplier.

### Normative shape

`PurchaseProposal`

-> `PurchaseAgreement/Order`

-> `SupplierFulfillmentObligation(supplier -> business)`

+ `PaymentObligation(business -> supplier)`

### Domain ownership

- supplier party identity: Identity;
- purchase agreement/order semantics: Commerce/Supply Chain procurement boundary;
- receiving/inventory: Supply Chain;
- payment execution/reconciliation: Finance;
- approval power: Governance/authority subsystem;
- books: future Accounting kernel.

### Execution shape

1. Authorized procurement intent is created.
2. Current institutional power/approval policy is evaluated independently of workflow state.
3. Purchase order/agreement is established by the owning domain.
4. Supplier delivery/performance is observed and accepted by Supply Chain or the owning work domain.
5. Supplier bill is related to the exact purchase obligation.
6. Finance executes payment only after current payment/approval policy qualifies the action.
7. Settlement is reconciled to the exact supplier payment obligation.
8. Accounting consumes accepted receipt/liability/payment events.

### Closure predicate

Procurement MAY close when:

- supplier fulfillment is accepted or otherwise resolved;
- payment obligation is reconciled/settled or validly waived;
- required receiving/inventory effects are accepted;
- required accounting state is accepted;
- disputes/returns/unknown effects are resolved for the chosen closure class.

### Adversarial vectors

- supplier bill without matching purchase order;
- received quantity differs from billed quantity;
- duplicate bill;
- payment approval expires between proposal and execution;
- goods received after payment cancellation;
- supplier identity changes;
- provider payment outcome unknown.

---

## GP-004 — Employee/contractor compensation

### Intent

Compensate a worker for an accepted work/compensation period.

### Normative shape

`EmploymentOrContractAgreement`

-> `WorkObligation(worker -> organization)`

+ `CompensationObligation(organization -> worker)`

+ role/power grants as applicable

### Domain ownership

- worker identity/credentials: Identity;
- employment/role authority: workforce + Governance/authority boundary;
- time/work evidence: owning workforce/work domain;
- payroll/tax determination: provider-integrated future boundary;
- payment: Finance;
- books: future Accounting kernel.

### Execution shape

1. Valid employment/contract agreement establishes compensation terms.
2. Work/time evidence is recorded by the owning domain and remains distinguishable from approval.
3. Current compensation approval power is evaluated.
4. Payroll/tax provider observations are retained as externally sourced determinations unless locally requalified by policy.
5. Finance executes authorized net/gross/remittance payments as applicable.
6. Settlement observations are reconciled to exact compensation/remittance obligations.
7. Accounting consumes accepted wage/tax/payment events.

### Closure predicate

A compensation period MAY close only when all policy-required obligations for that period are either:

- satisfied and reconciled;
- explicitly deferred under valid policy;
- waived/superseded where legally/institutionally permissible;
- disputed and classified accordingly.

### Adversarial vectors

- disputed hours;
- worker terminated before payment executes;
- approver revoked after approval but before execution;
- provider tax result unavailable;
- payment partially settles;
- duplicate payroll execution;
- correction discovered after settlement.

Compensation correction MUST be modeled as new causal records, not history rewriting.

---

## GP-005 — Refund and dispute

### Intent

Resolve a disputed commercial outcome and, where justified, return value or otherwise compensate.

### Normative shape

`Claim`

+ `CounterClaim`

+ `EvidenceSets`

-> `ResolutionPolicy`

-> `Resolution`

-> optional `RefundObligation` / replacement / credit / other compensating commitment

### Domain ownership

- dispute identity/participants: Identity;
- commercial subject/order: Commerce;
- delivery evidence: Supply Chain;
- payment/refund execution: Finance;
- decision authority: owning dispute/governance policy boundary;
- books: future Accounting kernel.

### Execution shape

1. Original transaction history remains immutable/retained.
2. Claim and counterclaim are recorded distinctly.
3. Evidence from Commerce, Finance, Supply Chain, communications, and external providers remains source-attributed.
4. Authorized resolution process produces an explicit resolution.
5. Resolution may create new obligations such as refund, replacement, credit, fee reversal, or no-action disposition.
6. Finance/Supply Chain execute compensating actions through their own authority boundaries.
7. Resulting effects are observed/reconciled independently.
8. Accounting records compensating economic events without deleting original entries.

### Closure predicate

The dispute MAY close when:

- an authorized resolution exists;
- every obligation created by that resolution is satisfied, waived, or explicitly remains open under the resolution;
- required compensating external effects are reconciled or classified as unresolved;
- appeal/supersession windows required by policy are represented correctly.

### Adversarial vectors

- carrier says delivered, customer says not received;
- refund request duplicated;
- refund outcome unknown;
- chargeback arrives after internal refund;
- resolution authority revoked;
- new evidence appears after resolution;
- accounting correction required after period close.

---

## GP-006 — Month-end close

### Intent

Produce an auditable accounting close for a defined period from accepted economic facts.

### Normative shape

`AcceptedEconomicEvents`

+ `AccountingPolicy(versioned)`

+ `Adjustments(authority/evidence)`

-> `Journal`

-> `Ledger`

-> `TrialBalance`

-> `Statements`

-> `CloseReceipt`

### Domain ownership

- source economic facts: Commerce, Finance, Supply Chain, workforce/asset/other owning domains;
- external bank/provider observations: their adapters;
- reconciliation: owning Finance/accounting reconciliation boundary;
- accounting recognition/journal/ledger: future Accounting kernel;
- close authority: Accounting + organizational policy/authority boundary;
- Business: task/projection only.

### Execution shape

1. Accounting imports only accepted/reconciled economic events or explicitly classified unresolved items according to policy.
2. Accounting policy version applicable to the period is fixed for the close candidate.
3. Reconciliation gaps remain visible.
4. Human adjustments/accruals/depreciation/corrections require explicit authority, reason, evidence, and causal references.
5. Trial balance and statements are deterministic projections of the accepted journal/ledger state under the selected policy.
6. Close receipt binds period, policy version, included event set/commitment, adjustment set, unresolved exceptions, and authority/evidence.

### Closure predicate

A period MAY be classified closed only under a named close policy whose predicates are satisfied.

If policy permits a close with exceptions, those exceptions MUST remain explicit in the closure receipt and projections.

`Closed` MUST NOT mean "all external reality is perfectly known." It means the exact institutional close policy was satisfied over the exact accepted evidence set.

### Adversarial vectors

- late bank observation;
- duplicate economic event;
- provider settlement reversal after close;
- stale accounting policy;
- unauthorized adjustment;
- missing source-domain evidence;
- late supplier bill;
- reopened/adjusted prior period.

Reopening or adjusting a period MUST create explicit supersession/correction lineage rather than rewriting the historical close silently.

---

# Cross-path qualification vectors

All six paths MUST eventually be tested under:

- stale authority;
- revoked authority;
- delayed evidence;
- contradictory observations;
- duplicate delivery/replay;
- provider reversal;
- clock skew;
- domain outage;
- restart/recovery;
- policy change during workflow;
- actor removal during workflow;
- malformed evidence reference;
- schema/version drift;
- compensation failure;
- unresolved dispute;
- export/reimport of causal history.

## Global pass condition

The v0.1 ontology passes its semantic gate only if all six paths can be expressed without:

1. adding another foundational primitive;
2. letting Business orchestration mint domain authority;
3. letting a projection strengthen uncertainty into truth;
4. treating compensation as rollback;
5. discarding causal/evidence history;
6. requiring universal disclosure of linked domain state.

If any path requires one of those shortcuts, this corpus MUST remain provisional and be revised before runtime implementation.
