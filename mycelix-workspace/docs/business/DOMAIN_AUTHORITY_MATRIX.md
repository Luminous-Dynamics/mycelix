# Domain Authority Matrix v0.1

Status: **normative candidate / documentation only**

This matrix defines which Mycelix domain is allowed to establish which kind of consequential truth for the Business substrate.

The matrix is intentionally conservative. A future adapter MAY narrow or add verification requirements. It MUST NOT widen another domain's authority simply for convenience.

## 1. Ownership rule

For every consequential Business-facing field, one of the following MUST be true:

1. one authoritative domain owns the fact; or
2. the fact is explicitly derived from multiple independently owned facts through a named reconciliation/resolution policy.

Business orchestration itself is not the default owner of any underlying domain truth.

## 2. Core matrix

| Institutional fact | Authoritative owner | Business may do | Business must not do |
| --- | --- | --- | --- |
| Person/agent identity | Identity | reference accepted identity state | mint or strengthen identity claims |
| Organization identity reference | Identity / lawful-identity adapter as applicable | reference organization identity and legal-entity evidence | pretend Mycelix organization identity is government legal status |
| Credential validity / assurance | Identity | consume typed current result | infer validity from stale cached credential material |
| Constitutional/governance policy | Governance | reference current policy and outcomes | create, bypass, or rewrite policy |
| Role/delegation institutional power | Governance/authority subsystem | request current power evaluation | treat UI role strings as authority |
| Commerce offering/catalog semantics | Commerce | reference offering/order/agreement state | invent commercial acceptance or fulfillment |
| Order/agreement commercial state | Commerce | coordinate follow-on workflows | mark accepted/closed without Commerce-owned outcome |
| Treasury/internal value authority | Finance | request authorized operation | create or widen financial authority |
| External payment execution | Finance + external provider adapter | retain request/result causality | treat provider availability as payment authority |
| Provider settlement observation | Finance adapter boundary | record observation | equate observation with reconciled settlement |
| Reconciled payment settlement | Finance/reconciliation policy | consume accepted settlement | self-declare paid from a callback |
| Inventory quantity/custody | Supply Chain | reference current accepted stock state | manufacture stock truth |
| Fulfillment/provenance | Supply Chain | observe accepted fulfillment state | mark delivered from orchestration alone |
| Procurement operational state | Supply Chain / Commerce boundary as defined by workflow | coordinate PO/bill/payment chain | collapse supplier obligation and payment into one Business flag |
| Work/job/project operational state | owning Work/Supply Chain/service domain | reference work outcomes | self-certify work completion |
| Payroll calculation/execution | provider-integrated workforce/Finance boundary, future | coordinate approved compensation workflow | implement jurisdictional payroll authority by default |
| Accounting recognition | future Accounting kernel | consume accepted economic events and policy | decide whether external/domain events happened |
| Journal/ledger/statements | future Accounting kernel | display projections | mutate authoritative books in Business read model |
| Tax determination | jurisdiction/tax policy adapter, future | pass transaction facts and consume determination | hard-code tax truth into Business/Commerce core |
| Legal registration/license status | authoritative registry/provider adapter + lawful-identity acceptance policy | retain verified observation/evidence | become the government registry |
| Communications | Pulse/owning communication domain | attach context refs to messages | make message delivery imply underlying business completion |
| Audit/compliance projection | derived from domain evidence + compliance policy | produce projection | invent evidence absent from source domains |
| Workflow coordination | Business Workflow Fabric | request typed actions, retain causality, evaluate closure | impersonate any authority-owning domain |
| Business Cockpit task/read model | Business projection layer | summarize accepted state | strengthen, erase, or silently reconcile uncertainty |

## 3. Three kinds of external fact

External systems commonly expose facts that must remain semantically distinct.

### 3.1 External observation

Example: PSP says a payment settled.

Owner: provider adapter records the observation.

Meaning: the provider asserted a fact under a specific source identity and time/evidence context.

### 3.2 Accepted domain fact

Example: Finance accepts the PSP observation as structurally/authentically valid under current provider policy.

Owner: Finance/provider-verification boundary.

Meaning: Mycelix accepts that this exact provider made this exact supported claim.

### 3.3 Reconciled institutional fact

Example: the accepted PSP settlement plus bank observation corresponds to PaymentObligation X.

Owner: Finance/reconciliation policy.

Meaning: the institution accepts that the relevant obligation has the resulting settlement state.

Business projections MUST distinguish these stages when the distinction matters.

## 4. Business-owned state

The Business layer MAY own state that is purely orchestration/projection state, including:

- workflow identifiers;
- workflow templates/profile identifiers;
- causal references between domain operations;
- task/read-model preferences;
- closure-policy identifiers;
- closure receipts derived from referenced domain outcomes;
- non-authoritative BusinessProfile composition/configuration;
- integration routing/configuration that does not itself grant domain authority.

It MUST NOT turn copies of domain records into a competing source of truth.

## 5. `Party` vs relationship projections

Identity owns accepted identity/persona facts.

Business MAY derive contextual relationship projections such as:

- Customer;
- Supplier;
- Employee;
- Contractor;
- Member;
- Lender;
- Borrower;
- Partner.

These labels SHOULD be derived from underlying agreements, commitments, powers, and domain activity rather than represented as independent duplicate identities.

A single `PartyRef` MAY have several simultaneous relationship projections.

## 6. Agreements and obligations

The domain that owns the substantive agreement owns the authoritative agreement state.

Examples:

- retail/service commercial agreement -> Commerce;
- employment governance/role authority -> workforce/Governance boundary;
- payment obligation settlement -> Finance reconciliation;
- delivery obligation satisfaction -> Supply Chain;
- constitutional membership obligations -> Governance.

Business MAY reference these commitments/obligations and evaluate workflow closure over them. It MUST NOT self-satisfy them.

## 7. Accounting authority

Accounting is deliberately separated into two questions:

1. **Did an economic event happen?** — answered by the owning operational domain and reconciliation boundaries.
2. **How should that accepted event be recognized in the books?** — answered by Accounting policy/kernel.

Therefore:

`Commerce/SupplyChain/Finance/... -> accepted EconomicEvent -> AccountingPolicy -> Journal -> Ledger`

Accounting MUST NOT become the authority source for whether the underlying sale, delivery, payment, wage accrual, or asset event occurred.

## 8. Authority evaluation shape

A consequential institutional transition SHOULD ultimately require a shape equivalent to:

`typed intent`

+ `current operation capability`

+ `current institutional power`

+ `current policy/constraints`

+ `action-specific facts`

-> `domain-owned authorization decision`

-> `domain execution`

-> `domain outcome/evidence`

The Business layer may assemble the request context but does not own the positive decision unless the operation is itself purely Business-owned orchestration state.

## 9. Ambiguity rules

### 9.1 Missing data

Missing/unavailable domain data is not negative truth.

### 9.2 Conflicting sources

Conflicting observations remain distinct until reconciled or resolved under explicit policy.

### 9.3 Stale data

A cached accepted fact MUST NOT be presented as current when currentness is required and its freshness horizon has expired or is unknown.

### 9.4 Provider disagreement

Provider disagreement is not resolved by "latest timestamp wins" unless an owning-domain policy explicitly establishes that rule and the timestamps themselves are trusted for that purpose.

### 9.5 Workflow disagreement

A workflow never chooses a domain winner merely to progress. It waits, fails closed, requests reconciliation, or enters an explicit dispute/compensation state.

## 10. Future matrix maintenance

Every new Business-facing field or workflow SHOULD answer:

- What domain owns the underlying truth?
- What exact evidence justifies the Business projection?
- What freshness boundary applies?
- What happens when the source is unavailable?
- What happens when sources conflict?
- What authority may change the state?
- What exit/export representation preserves the meaning?

If these answers are absent, the field is not ready to become consequential Business state.
