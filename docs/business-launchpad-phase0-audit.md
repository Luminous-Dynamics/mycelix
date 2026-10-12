# Business Launchpad — Phase 0 repository audit

**Audit date:** 2026-10-12  
**Status:** Initial source/documentation audit; incomplete  
**Audited Mycelix main commit:** `445a84abdaa64f3d05e2d5c51115e4d060b78ec7`  
**Audited Symthaea main commit:** `b9af191f380d635f0459b24284c1d1fd6a5516aa`  
**Audited Luminous Platform main commit:** `244be246c54a7f55f38fb87055aa362258e020fa`  
**Scope limitation:** This was remote source inspection through GitHub, not a clean local clone/build, full dependency/license audit, runtime test, security audit, or legal opinion.

## 1. Executive decision

Proceed with a small, separate business-facing product layer and a deterministic business-economics core. **Do not build a replacement ERP, payment system, cooperative currency, or universal compliance engine as the first product.**

There is enough reusable groundwork to avoid starting entirely from scratch: Mycelix Marketplace has a guarded listing/purchase/fulfillment model, Commons has food-production/distribution concepts, and Luminous Platform provides a separate host/runtime layer. There is not yet evidence from this audit of an integrated merchant application that completes onboarding, viability assessment, order/job operations, payment reconciliation, bookkeeping and local compliance end-to-end.

The first code slice committed on the Business Launchpad branch is therefore deliberately narrower: a pure calculation module using integer minor currency units, transparent assumptions, explicit omitted costs, and deterministic tests. It does not call Symthaea, interact with Holochain, move money, persist business records, or decide legal compliance.

## 2. Repository evidence

### Mycelix repository

Audited `main`: `445a84abdaa64f3d05e2d5c51115e4d060b78ec7`.

| Area | Evidence inspected | Finding | Business Launchpad implication |
|---|---|---|---|
| Marketplace product | [README](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-marketplace/README.md), blob `3f177984bbb127e56b5aadde869e478698f262cc` | Describes itself as research-grade/pre-alpha; explicitly does not establish production readiness, general Byzantine tolerance, legal finality, or global convergence. Documents a Leptos migration path and a set of listing and transaction operations. | Reuse where the existing contract fits; do not present it as a production commerce platform before the exact runtime evidence is verified. |
| Marketplace purchase contract | [vertical-slice contract](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-marketplace/contracts/leptos-vertical-slice-v1.json), blob `09f262f5ca95ca73a813cb638007aba90bfa3fe3` | Browse/listing/purchase/recovery slice. Requires authoritative listing terms, buyer authorship, immutable transaction terms, integrity-layer lifecycle validation, and no automatic fixture fallback. | Make this contract the starting compatibility boundary for Marketplace reuse. |
| Marketplace lifecycle contract | [lifecycle contract](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-marketplace/contracts/leptos-lifecycle-slice-v2.json), blob `db9466e4fe3f79b8aeb32653ae1431fe32dceb5f` | Allows seller confirmation/shipping, buyer delivery confirmation, and authorized cancellation in the described frontend slice. Settlement and dispute mutations remain deferred behind explicit gates. | Preserve gates. Do not turn an order into “paid,” “disputed and resolved,” or “refunded” merely because a frontend or local transaction says so. |
| Marketplace conflict policy | [policy contract](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-marketplace/contracts/transaction-conflict-policy-v1.json), blob `0d4871856ba0b74d61f3a62dee046bee332bf923`; [resolution contract](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-marketplace/contracts/transaction-conflict-resolution-v1.json), blob `d2280d62fc2e44036d5564fb1a6633aee7f1edcc` | Policy says choose only an existing authored head under narrow safety/authority conditions; unsafe concurrent heads remain conflicted. | Do not introduce a second timestamp-based transaction reducer. Confirm current file hash and behavior at the integration commit before depending on exact policy content. |
| Marketplace claims | [Claims/evidence ledger](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-marketplace/docs/CLAIMS_AND_EVIDENCE.md), blob `6778d8ad88eb4a2c8c0cd3fee320af421349a96e` | Explicitly limits what may be claimed from static contracts, unit tests, controlled two-conductor evidence, and settlement projection. | Retain the existing evidence ladder. Static-checker success is not conductor execution or production proof. |
| Commons food domain | [Commons README](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-commons/README.md), blob `ae17e7ea7e77d62c738dcf48373c3ec5c769e623` | Has `food-production`, `food-distribution`, `food-preservation`, and `food-knowledge` domains. The README lists unit and ignored conductor test commands. | Reuse for food production/distribution coordination where validated. It is not a retail POS, stock ledger, complete restaurant system, or regulatory authority. |
| Finance | [Finance README](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-finance/README.md), blob `ff489a33f29932cc9eae5df57b99a9c807ee670d` | Defines MYCEL reputation, SAP circulation and TEND time exchange, plus treasury/payment/bridge zomes. | Do not conflate these economic primitives with bank/card processing, statutory currency, accounting ledger postings, regulated lending, or external settlement evidence. Keep merchant economics isolated from the DAO currency formulas. |
| SDK economics | [economics module](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/sdk/src/economics/mod.rs), blob `bced47a88852e609684e4ab788003bad3942c78a` | Exposes the existing Mycelix community-economics and currency types. | Avoid adding merchant break-even calculations to this module: it would couple basic business math to community currency/tokenomics and an SDK with its own packaging/dependency context. |
| Existing Observatory food UI | [food route](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/observatory/src/routes/food/+page.svelte), blob `09e78c41a9e6c99ec9e04de5820caf1ad11d723e` | Current interface models plots, harvests, resource inputs and nutrient summaries. | Useful possible data-source/product sibling, but it is not an owner-facing food retail/restaurant workflow. |
| Existing budget UI | [budgets route](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/observatory/src/routes/budgets/+page.svelte), blob `24f4b87591d10b68e821bfde1c9427c8360e69c9` | Tracks participatory/community budget cycles in SAP. | Not personal business cash flow, a general ledger, or a merchant funding forecast. |
| Tax export | [tax export module](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/observatory/src/lib/tax-export.ts), blob `a0351f23ec6cfe74efc78d91651f67a1e34b0f56` | Comments describe SARS-compliant exports for TEND exchanges and a legacy carve-out accepting some DID formats. This audit did not verify legal basis, completeness, or current correctness. | Flag this for a separate legal/source review. Do not reuse the “SARS-compliant” claim in Launchpad marketing or merchant filings without independent verification. |

**Note:** These are Git blob IDs captured during inspection, not independent third-party attestations. Re-fetch files at the chosen exact branch/commit before release.

### Symthaea

Audited `main`: `b9af191f380d635f0459b24284c1d1fd6a5516aa`.

The [README](https://github.com/Luminous-Dynamics/symthaea/blob/main/README.md) describes an experimental cognitive architecture and explicitly scopes cryptographic primitive demonstrations, measured runtime/benchmarks, and moral/cognitive metrics. It documents corrected/retracted internal benchmark claims and says the system does not establish consciousness or general regulatory compliance.

**Decision:** Use Symthaea behind an optional, replaceable advisory interface. The first deterministic viability calculation must not require it. Any later business forecasting needs its own time-ordered test corpus, baselines, calibration/error reporting, abstention behavior, and versioned input/output provenance. General Symthaea performance measurements are not evidence that sales forecasts or business recommendations are accurate.

### Luminous Platform

Audited `main`: `244be246c54a7f55f38fb87055aa362258e020fa`.

The [Platform README](https://github.com/Luminous-Dynamics/luminous-platform/blob/main/README.md) describes sovereign boot, NixOS management, operations/fleet intent, a meta-flake, and networking/runtime components. Its safety boundary says platform components may observe host state but must not be required for host boot; the operational components have explicit non-actuation/consent constraints.

**Decision:** Keep the merchant-facing UI/service and business domain model outside the host/OS runtime. Consume platform security and deployment interfaces only where stable interfaces exist; do not put orders, invoices, or tax logic into Nixward or sovereign-boot.

## 3. Executable slice added on the RFC branch

Paths:

- [Core](https://github.com/Luminous-Dynamics/mycelix/tree/rfc/business-launchpad-architecture/mycelix-workspace/business-launchpad/core)
- [Source](https://github.com/Luminous-Dynamics/mycelix/blob/rfc/business-launchpad-architecture/mycelix-workspace/business-launchpad/core/src/index.js)
- [Test fixtures](https://github.com/Luminous-Dynamics/mycelix/blob/rfc/business-launchpad-architecture/mycelix-workspace/business-launchpad/core/test/fixtures/business-scenarios.v1.json)
- [Test workflow](https://github.com/Luminous-Dynamics/mycelix/blob/rfc/business-launchpad-architecture/.github/workflows/business-launchpad-core.yml)

Current responsibilities:

- Integer minor-unit money representation with `BigInt` intermediates and safe-integer overflow rejection.
- Unit contribution, monthly scenario arithmetic, break-even units, required startup cash, and simple funding gap.
- Explicit incomplete-cost declarations and named omissions.
- Three frozen synthetic scenario fixtures: service, food and retail.
- Deterministic Node.js tests and an exact-head, version-pinned GitHub Actions workflow.

The local scratch checkout ran `node --check src/index.js` and `node --test`: 19 tests/subtests passed on Node v22.16.0. This is useful local evidence for the equivalent source and fixtures, not an exact-head CI pass. The pinned workflow targets Node 24.21.0; its run must complete against the current branch head before claiming that result.

The example numbers are invented test values in ZAR minor units, not gathered Johannesburg prices, cost benchmarks, profit projections, or recommendations. The calculated “operating result” excludes any omitted costs and is not net profit. The current code does not calculate a time-based cash runway, full working-capital cycle, creditworthiness, local taxes, or demand.

## 4. Current hard gaps

1. **No user-facing business product.** There is no integrated onboarding, business dashboard, persistence model, accessible low-bandwidth UI, or end-to-end owner journey in this slice.
2. **No single source of truth for merchant transactions.** Marketplace is the reuse candidate for its supported commerce lifecycle; service quotes/jobs may need a distinct typed model. This choice has not been implemented or integration-tested.
3. **No external accounting/POS/payment adapter.** No sandbox operation, webhook verification, reconciliation, or actual external payment evidence was executed in this audit.
4. **No jurisdiction pack.** The calculator has no official rules or tax rates; current South African VAT source status is nuanced and must not be reduced to a static threshold.
5. **No real price/cost dataset.** Synthetic fixture inputs test arithmetic only. Local market research is required before any concrete viability advice is given.
6. **No local build of Marketplace or Holochain.** The relevant static-checker, Rust/WASM build, conductor, and multi-agent commands were inspected but not run in this remote repository inspection.
7. **Licensing/reuse needs review.** The root tree contains different licensing declarations: Mycelix Marketplace identifies AGPL-3.0-or-later, while Finance identifies Apache-2.0, and the SDK labels AGPL-3.0-or-later. Evaluate exact files, dependencies, deployment and distribution model before packaging an integrated product.
8. **No security/privacy review.** Tenant isolation, owner/admin delegation, local/offline storage, secrets management, backup/restore, deletion, breach response and personal-information obligations are not yet implemented in Launchpad.
9. **No market-fit evidence.** Neither these calculations nor the presence of existing Mycelix domains demonstrate that a shop can start faster, survive longer, or earn more using this product.

## 5. External source findings relevant to the South African pilot

### VAT and small-business tax

SARS's [VAT Connect Issue 21 (September 2026)](https://www.sars.gov.za/businesses-and-employers/my-business-and-tax/vat-connect-issue-21-september-2026/) says the 2026 Draft Rates Bill **proposes** raising the compulsory VAT registration threshold from R1 million to R2.3 million and the voluntary threshold from R50,000 to R120,000 from 1 April 2026. SARS says it is applying the R2.3 million compulsory threshold for new registration/deregistration requests and that proposed amendments will be retrospective once promulgated; it also says registrations are not automatically cancelled at this point. SARS's [Budget 2026 FAQ](https://www.sars.gov.za/about-sars-tax-and-customs-system/budget/budget-2026-frequently-asked-questions/) separately explains the thresholds.

Do not turn those pages into a timeless hard-coded rule. The pack must capture source status, date checked, registration context, applicability, and current promulgation status, and direct uncertain cases to official guidance or qualified tax advice. This audit does not establish the latest gazetted statutory state for every taxpayer circumstance.

### Food business

South Africa's National Department of Health lists [Food Control Regulations](https://www.health.gov.za/food-control-regulations/); Regulation R638 covers general hygiene requirements for food premises and food transport. Applicability also depends on the activity, location and municipality; the Launchpad must not imply that a generic company-registration step, an uploaded certificate, or a national-level checklist establishes all required permissions.

### Existing accounting/POS systems

ERPNext's official documentation describes [POS with inventory integration](https://docs.frappe.io/erpnext/point-of-sale), [inventory accounting](https://docs.frappe.io/erpnext/accounting-of-inventory-stock), and [accounting entries](https://docs.frappe.io/erpnext/accounting-entries). These are enough to justify an integration spike before building duplicate POS, stock and ledger features. They do not establish that ERPNext is the right choice for all businesses, compatible with the final distribution model, usable offline for this pilot, or easy enough for a nontechnical microbusiness. Those require a disposable deployment, licensing review, adapter contract, export test and operator walkthrough.

## 6. Next work in dependency order

1. **Finish exact-head local validation of the isolated core:** fetch current branch head; run package checks and tests; compare local and Actions result. Treat queued as not passed.
2. **Add mutation-free CLI/example interface or UI spike:** render a sample report and the explicit limitations; do not mix business assumptions with Mycelix currency/tokenomics.
3. **Do an ERPNext disposable adapter spike:** import a synthetic catalog and expense; create and reconcile a synthetic order/invoice/payment record; verify IDs, totals, export and retry semantics. No real provider credentials.
4. **Choose the first market slice:** one municipality and one business activity, with source-reviewed official prerequisites. Do not call it “global compliance”.
5. **Only after the above, build the business-facing UI**, keeping the calculator deterministic and Symthaea advisory-only.
6. **Integrate Marketplace/Mycelix only for proven shared semantics**, after reproducing the current vertical-slice/lifecycle and multi-agent evidence at an exact commit.
7. **Pilot with explicit opt-in and measurement:** compare setup time/cost and administrative burden against a baseline. Do not claim success from simulation, README counts, or feature presence.

## 7. Decision

**Continue, but keep scope narrow.** There is a credible reuse path through Mycelix Marketplace and Commons, with Symthaea for optional advice and Luminous Platform for underlying secure infrastructure. The current truth is that we have added a deterministic calculator with synthetic fixtures and tests—not an integrated business-launch product yet.

The next meaningful proof is not another architecture diagram. It is an exact-head test pass, one reproducible integration spike, a source-reviewed local prerequisite pack, and one complete owner-facing flow that records what happened without inventing payments, compliance, or profitability.
