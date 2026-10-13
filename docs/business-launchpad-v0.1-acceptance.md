# Business Launchpad v0.1 — Acceptance and Evidence Matrix

**Status:** Proposed acceptance contract. A deterministic calculator core, CLI and static browser worksheet exist; the workflow is not integrated with persistent records, Marketplace transactions, Symthaea or external accounting/payment systems. No exact-head CI pass is being claimed.  
**Companion:** [Business Launchpad architecture RFC](business-launchpad-architecture.md)  
**Date:** 2026-10-12  
**Purpose:** Turn the RFC into testable acceptance criteria before code is assigned to an existing repository or a new application.

## 1. Release boundary

v0.1 is an owner-facing business setup and operating workflow with a no-real-money vertical slice. It may use mock data and a disposable external-system adapter in automated tests, but it must not move real customer funds, issue loans/stored value, file legal/tax documents, or automatically commit a consequential business decision.

The minimum complete journey is:

1. Create a private business profile.
2. Enter costs, prices, and assumptions.
3. Compute a transparent break-even and runway worksheet.
4. Track a quote/order/job through an authorized decision and recorded outcome.
5. Record revenue/expenses and payment evidence without overstating settlement.
6. Export business records and inspect the audit trail.
7. See applicable launch prerequisites, source freshness, and unresolved blockers.

A service-business journey is a reasonable first end-to-end slice because it avoids needing an inventory/accounting engine on day one. The domain model must still keep extension points for food and retail. The first municipality and business activity remain explicit configuration decisions; do not imply global compliance coverage in v0.1.

## 1.1 What exists in the current branch

An isolated calculator module lives at [`mycelix-workspace/business-launchpad/core/`](../mycelix-workspace/business-launchpad/core/). Its current responsibilities are a single-offer economics worksheet and a separate bounded cash-liquidity scenario:

- contribution per unit and monthly illustrative totals using integer minor currency units;
- upward-rounded break-even unit calculation, or explicit `unreachable` when contribution is not positive;
- setup cost + working-capital requirement and the arithmetic gap after available startup cash;
- a separate 12-month editable cash scenario with independent monthly sales activity, sales cash receipts, operating cash payments and other inflows/outflows; setup costs are deducted before Month 1 and the reserve threshold, reserve/negative-balance breaches, lowest projected cash and additional reserve buffer are reported;
- explicit partial-cost coverage and named omissions;
- safe-integer/overflow rejection and deterministic amount formatting.

It has synthetic service, food and retail fixtures in `test/fixtures/business-scenarios.v1.json`, a local JSON CLI in `src/cli.js`, and a static browser client in `../web/` that imports the same core. Run `npm run example` for the CLI, or serve `mycelix-workspace/business-launchpad/` on loopback and open `/web/`. The browser prototype supports explicit local JSON import/export for assumptions and calculated reports, including the month-by-month cash plan; inputs are not uploaded or stored in browser storage. Changing any input invalidates the previous report until recalculation. There is no persistent server-side record system. Cash receipts and operating cash payments can be shifted between months independently; starter values are only convenience assumptions derived from single-offer economics, and legacy version-1 plans get same-month-derived values for missing fields. This does not provide receivable/payable ledgers, invoice aging or reconciliation against actual settlements. Capacity, inventory-specific costing, seasonality and demand prediction are not modeled. Fixture/example values are illustrative, not researched local prices or recommendations.

The temporary workspace-shaped scratch checkout ran syntax checks for the calculator, CLI and browser module; `npm test` reported **27 tests/subtests passed** on Node v22.16.0; the CLI example returned expected arithmetic (break-even 8 units/month; startup funding gap 0 minor units for fictional inputs). Six dependency-free mock-DOM tests simulated the browser client's default calculation, excess decimal precision, complete-cost selection, negative contribution, malformed input, and source scan for browser persistence/network APIs. This is an interaction simulation, not a real-browser or accessibility test. Chromium attempts were blocked by this execution environment's managed URL policy for both loopback HTTP and file URLs; that is an environment blocker, not evidence that the app itself fails. The scratch files are not byte-identical to the current GitHub branch files (local source blob `60e83e62b62cbd8c9f4bde5ea8bafa4bc36738c4` vs branch source blob `76b1f4fd70f056b1c051fb62b5368f08c1a972cc`; local core-test blob `779c00a552a41bb04765623ccbdab3c2e4eb68f1` vs branch test blob `d14d90d3522afd94d370da62f0fad5e11881a14e`; local mock-DOM test blob `ebb5e588e7ab8459165ff9d443dafe27210aa0c6` vs current branch mock-DOM blob `fd31879972040e4cbad0bb7607ed95c7be9823e9`). Treat this as **local algorithm/fixture/interaction-simulation evidence only**, not exact-commit verification. Five newer tests for scenario JSON import/export and invalidating stale reports were added after this scratch run and have not yet been executed locally. The pinned Node 24.21.0 GitHub Actions workflow must pass against the current exact head before the claim ceiling rises. Responsive layout, real-browser behavior and accessibility remain unvalidated. The static client is not a persistent database, production integration, jurisdiction rules engine, external payment processor, POS or accounting adapter.

## 2. Test classes and evidence levels

Every criterion is labeled with the first evidence level needed:

- **Static contract:** source/spec agrees on names, fields and intended boundary.
- **Unit/property:** deterministic function or state transition behaves correctly.
- **Build:** exact commit builds under the locked toolchain.
- **Integration:** actual components or adapter interact.
- **End-to-end:** independent actors and complete workflow execute.
- **Adversarial:** retries, malformed input, collusion, race, outage, and unauthorized access are tested.

A checker that searches source text or JSON markers is useful for drift detection, but it cannot establish runtime behavior. Every test result must bind to the exact commit, dependency/toolchain versions, test inputs, and relevant artifact hashes. Do not promote a criterion to a higher evidence level because a lower-level check passed.

## 3. Acceptance matrix

### A. Business setup and policy source quality

| ID | Scenario | Required assertion | First evidence |
|---|---|---|---|
| BL-SET-001 | Create a business profile | Profile has a stable ID, jurisdiction, business type, currency, timezone, owner/capability assignments, and creation event. The profile remains private unless the owner explicitly publishes a specific listing/claim. | Unit + integration |
| BL-SET-002 | Unauthorized access to another business | A user without a valid capability cannot read, change, export, or enumerate the other business. UI hiding alone does not satisfy the test. | Adversarial integration |
| BL-SET-003 | Unknown legal form | The product explains options and identifies what information is missing; it does not silently select a legal structure or claim the owner is registered. | Unit + end-to-end |
| BL-SET-004 | Jurisdiction source is stale | A consequential compliance step remains blocked or clearly marked unverified when its required pack has expired, been withdrawn, or lacks required applicability data. Informational content can remain visible with a warning. | Unit/property |
| BL-SET-005 | Two sources conflict | The pack surfaces the conflict, source dates, jurisdictional scope and unresolved status; it must not silently select the more permissive interpretation. | Unit + fixture |
| BL-SET-006 | Rule effective-date boundary | A rule that changes on date D produces the documented behavior before and after D using jurisdiction-local date semantics. Missing dates produce an unknown state. | Property + golden fixtures |
| BL-SET-007 | Food business setup | Food-specific steps are shown only when the business activity and premises make them applicable. An uploaded certificate is treated as evidence to verify, not as proof that every other permission is satisfied. | Integration + jurisdiction fixtures |
| BL-SET-008 | Owner corrects an input | The old calculation and recommendation remain traceable to the old input version; downstream current results are recomputed, not silently attributed to the previous data. | Unit/property |

### B. Economics worksheet

| ID | Scenario | Required assertion | First evidence |
|---|---|---|---|
| BL-ECO-001 | Positive contribution margin | For a simple single-product worksheet, contribution per unit = selling price minus variable cost per unit; break-even units = ceiling(fixed costs / positive contribution per unit). Currency and rounding rules are explicit. | Property/golden |
| BL-ECO-002 | Zero/negative contribution | No finite positive break-even is shown. The product reports the model as infeasible under current assumptions and shows the inputs that caused it. | Unit/property |
| BL-ECO-003 | Incomplete cost data | Unknown costs are not silently treated as zero. The output lists omissions and labels totals as partial. | Unit/property |
| BL-ECO-004 | Cash runway | Cash runway is estimated only from an explicit monthly plan. Setup cash, starting cash, and the reserve threshold are kept distinct; negative and below-reserve balances and the minimum cash point are reported. Each month reconciles operating, investing and financing net cash flows plus the explicit unclassified residual to total cash movement and closing cash. | Unit/property |
| BL-ECO-009 | Delayed customer collection | Sales units in Month 1 do not automatically become Month 1 cash when explicit receipt timing is supplied. An expected collection in Month 2 changes Month 2 cash without rewriting Month 1 activity. | Property/golden |
| BL-ECO-010 | Advance operating payment | An operating cash payment can occur before the related sales cash receipt. The resulting lowest balance and reserve breach reflect the payment's entered month. | Property/golden |
| BL-ECO-011 | Legacy cash-plan import | Version-1 plans missing explicit sales-receipt or operating-payment fields are normalized with clearly disclosed starter defaults; explicit valid fields are never overwritten. | Round-trip + exact-head |
| BL-ECO-012 | Separate variable-cost payment timing | Split the existing per-unit variable cost across three named categories whose shares total exactly 100%; each category uses its own three-bucket schedule totaling 100%. The three category costs reconcile exactly to the entered variable cost in minor units. Changing one category's payment timing changes cash timing without changing total unit cost, sales activity, or the owner-entered base plan. Per-category and total payments beyond Month 12 remain explicit. Legacy schema-v5 aggregate timing applies equally to all categories and preserves the previous aggregate timing. | Core/property + mock-DOM + exact-head |
| BL-ECO-013 | Actual versus forecast cash | For each imported, fully covered calendar month within the plan, display plan, actual and actual-minus-plan for all eight cash-flow buckets, net cash movement variance, opening-balance difference and closing-balance variance. Positive outflow variance means spend exceeded plan; closing-balance variance must remain distinguishable from net-movement variance because the opening balance may differ. Transaction totals must reconcile to the source range opening/closing balances. A hash does not authenticate the source. | Core/property + mock-DOM + Chromium + exact-head |
| BL-ECO-005 | Scenario sensitivity | Changing demand, price, or cost inputs reproducibly changes the displayed result. The output stores the scenario inputs and calculation version; changing cash timing changes liquidity without fabricating a change in sales activity. | Property/golden |
| BL-ECO-006 | Cash stress comparison | Base is exactly the owner-entered cash plan. Downside/upside transformations use persisted, bounded and visible assumptions; zero-stress settings reproduce the base projection; receipts delayed beyond the horizon are disclosed instead of counted as collected; the comparison stores all case projections and assumptions. No stress value is represented as probability, benchmark or forecast. | Core + Mock-DOM + exact-head Chromium |
| BL-ECO-009 | Terms-based timing schedule | Customer receipt and per-unit variable-cost payment allocations are integer percentages across current/+1/+2 months and each sum to 100%; the base plan is not overwritten; gross receipts and payable cohorts reconcile exactly using minor-unit allocation; fixed overhead stays same-month; receipts and payments beyond the horizon are separately disclosed; other/investing/financing movements are carried through unchanged. Do not claim invoice-level AR/AP or day-level due-date coverage. | Core property corpus + Mock-DOM + exact-head Chromium |
| BL-ECO-006 | Symthaea unavailable | Core deterministic calculations and record management continue to work. Model-dependent advice is clearly unavailable rather than fabricated. | Integration |
| BL-ECO-007 | Symthaea recommendation | Every recommendation includes input/data provenance, data window, model/rule version, assumptions, uncertainty/abstention state, and a way to compare the suggestion with a deterministic baseline. | Contract + integration |
| BL-ECO-008 | Insufficient data or out-of-distribution inputs | Symthaea may abstain; the UI makes the reason legible and does not convert low confidence into false precision. | Unit + evaluation corpus |

### C. Quote, order, job, and fulfillment

| ID | Scenario | Required assertion | First evidence |
|---|---|---|---|
| BL-COM-001 | Create a quote/job | The proposed terms include stable identity, exact item/service, quantity/scope, amount/currency, expiry, responsible parties, and revision. Proposal does not imply acceptance or payment. | Unit + integration |
| BL-COM-002 | Owner approves external effect | Approval binds to the exact reviewed payload/digest. Any material change invalidates approval and requires a new explicit decision. | Unit/property |
| BL-COM-003 | Reuse Marketplace lifecycle | For the listing/purchase/fulfillment use case, Launchpad uses the existing Marketplace transaction semantics when compatible. A separate service-job state machine is introduced only for a documented domain mismatch. | Contract + integration |
| BL-COM-004 | Concurrent edits | Conflicting transaction revisions are surfaced. No arbitrary “latest timestamp wins” rule may silently authorize a disputed material change. | Property + multi-agent |
| BL-COM-005 | Unauthorized lifecycle transition | Only the required actor can confirm, ship, deliver, or cancel according to the applicable contract. UI controls are not the authority boundary. | Integration/adversarial |
| BL-COM-006 | Cancellation/refund | Cancellation state is not misrepresented as a completed refund; refund evidence, if applicable, is a separate linked external effect. | Integration |
| BL-COM-007 | Offline order | The record is marked pending/unsynced. Resynchronization is idempotent; duplicate local messages do not create duplicate authoritative purchases. | Fault-injection integration |
| BL-COM-008 | Shipment/delivery evidence is absent | The UI cannot mark fulfillment as verified solely because a user opened a screen or pressed a button. Record whether evidence is owner-attested, counterparty-attested, or independently verified. | Unit + end-to-end |

### D. Payment, accounting evidence, and audit

| ID | Scenario | Required assertion | First evidence |
|---|---|---|---|
| BL-FIN-001 | Record manual income/expense | Amount, currency, date, category state, counterparty, and receipt/source reference are captured; unknown classifications stay unknown. | Unit + integration |
| BL-FIN-002 | Provider accepts request | Status becomes submitted/accepted only. The application does not claim funds settled until authoritative status confirms it. | Adapter contract |
| BL-FIN-003 | Timeout after submission | Status becomes indeterminate when the side effect may have reached the provider. Reconciliation queries authoritative status before any non-idempotent retry. | Fault injection |
| BL-FIN-004 | Duplicate webhook/provider event | Duplicate event IDs or idempotency keys cannot create duplicate payments or accounting effects. | Property + adapter integration |
| BL-FIN-005 | Out-of-order webhooks | Event order cannot regress a settled transaction into an earlier state. Provider source-of-truth status and valid transitions determine the projection. | Property + adapter integration |
| BL-FIN-006 | Amount/currency/reference mismatch | Evidence is quarantined for reconciliation; it cannot settle an unrelated order or a transaction with a different amount/currency. | Adversarial integration |
| BL-FIN-007 | Business export | Export includes all user-owned records and declared evidence references in a documented format; totals match the source dataset. Export does not leak another tenant's records. | Round-trip + adversarial |
| BL-FIN-008 | Correction to past transaction | A correction event references the prior record and reason. The audit history remains inspectable; no silent destructive rewrite. | Unit/property |
| BL-FIN-009 | Mycelix finance not active | The business workflow does not invent settlement evidence from SAP/TEND/MYCEL records or from an unavailable Finance role. | Contract + negative integration |
| BL-FIN-010 | Finance settlement is feature-gated | Marketplace Finance settlement is disabled unless the required Finance role, retry-safe Finance patch, and current live settlement evidence are all present as required by existing contracts. | Static + end-to-end promotion |

### E. Privacy, authorization, and operational resilience

| ID | Scenario | Required assertion | First evidence |
|---|---|---|---|
| BL-SEC-001 | Public listing creation | Only the fields explicitly selected for publication leave the private business store. Customer data, private prices, payroll, bank credentials, and internal notes are not published by default. | Integration + packet inspection |
| BL-SEC-002 | Credentials and secrets | No tokens, provider secrets, raw card data, passwords, or banking credentials appear in logs, URLs, crash output, public DHT entries, or ordinary audit messages. | Static + adversarial |
| BL-SEC-003 | Revoked capability | A revoked or expired capability cannot authorize new actions; cached UI state cannot bypass server-side revalidation. | Integration |
| BL-SEC-004 | Backup and restore | A clean restore preserves all committed records, event references, and export ability; the test records recovery point and recovery time. | End-to-end operations test |
| BL-SEC-005 | Network partition | The app remains clear about local versus synchronized state. It must not invent global uniqueness, availability, payment, or settlement under a partition. | Fault injection |
| BL-SEC-006 | Retry after restart | A restarted worker does not repeat a completed side effect; pending and indeterminate actions are recovered through explicit reconciliation. | Restart-recovery test |
| BL-SEC-007 | Data deletion request | The system follows its documented retention/legal-hold policy, removes or de-identifies applicable private data where required, and explains the limits of deleting previously shared public claims or audit evidence. | Integration + policy review |
| BL-SEC-008 | Recommendation disabled | Removing Symthaea credentials or turning off the advisory feature does not break business transaction authority, records, exports, or payment reconciliation. | Integration |
| BL-SEC-009 | Model-generated malicious input | The system treats generated text as untrusted data; a recommendation cannot execute a tool, open a payment, or change authorization without the domain protocol and owner approval. | Adversarial integration |

## 4. Existing Marketplace contracts to preserve

These current files are candidate compatibility contracts, not proof of live success:

- [leptos-vertical-slice-v1.json](../mycelix-marketplace/contracts/leptos-vertical-slice-v1.json) — browse, listing details, purchase creation, transaction detail/index, immutable purchase terms, buyer authorship, integrity-validated state transitions and no automatic fixture fallback.
- [leptos-lifecycle-slice-v2.json](../mycelix-marketplace/contracts/leptos-lifecycle-slice-v2.json) — conflict-aware reads, guarded seller confirmation/shipment, buyer delivery confirmation and authorized cancellation; settlement and dispute remain explicitly deferred.
- [transaction-conflict-policy-v1.json](../mycelix-marketplace/contracts/transaction-conflict-policy-v1.json) and [transaction-conflict-resolution-v1.json](../mycelix-marketplace/contracts/transaction-conflict-resolution-v1.json) — inspect current versions before depending on contract semantics.
- [CLAIMS_AND_EVIDENCE.md](../mycelix-marketplace/docs/CLAIMS_AND_EVIDENCE.md) and [STATUS_AND_CLAIMS.md](../mycelix-marketplace/docs/STATUS_AND_CLAIMS.md) — current claim boundaries and evidence levels.

Before coding an integration, verify the current contracts against the source at an exact commit. The existing static checkers are drift detectors and do not replace builds, relevant unit tests, conductor tests, independent multi-agent execution, or artifact-bound live promotion.

## 4.1 Portable scenario/report files in the static prototype

These criteria cover the current local-file workflow, including the month-by-month cash plan. The exact-head mock-DOM and core tests have been extended, but they still require a completed exact-head Actions run. A separate Chromium journey has also been added; inspect its exact-head artifacts before raising the evidence level.

| ID | Scenario | Required assertion | First evidence |
|---|---|---|---|
| BL-DATA-001 | Download assumptions | Download a schema-version-2 JSON envelope containing current normalized model inputs, the 12-month cash plan with explicit monthly sales receipts and operating payments, currency metadata and cost-coverage declaration. No input is posted to an application endpoint. | Mock-DOM + exact-head |
| BL-DATA-002 | Load valid assumptions | Accept schema versions 1–6; versions 2–6 require a 12-month plan, versions 4–6 require downside/upside stress assumptions, version 5 requires its aggregate timing schedule, and version 6 requires one collection schedule plus exactly three variable-cost categories with independent payment schedules. Version 1 without a plan receives an illustrative ramp and may derive missing receipt/payment fields; version 2 preserves timing and defaults new investing/financing amounts to zero; version 3 preserves categories and receives illustrative stress defaults; version 4 preserves stress settings and receives illustrative timing defaults; version 5 applies its aggregate supplier-payment curve to all three categories, preserving the previous timing. For schema 6, validate each schedule percentage as an integer from 0–100, require cost shares to total 100%, and require every three-bucket allocation to total 100%. Preserve the owner-entered base plan; the terms-based case derives gross receipts from units × price, allocates the entered variable cost by category shares, and independently times each category's supplier payments. Fixed overhead remains in its originating month, unrelated cash movements are unchanged, and horizon deferrals remain visible. Verify exact minor-unit conservation, zero-stress parity, and reconciliation to closing cash. Require recalculation before a report becomes downloadable. | Mock-DOM + exact-head |
| BL-DATA-003 | Load malformed, oversized or unsupported file | Show a visible error, do not change the user's current input fields, and keep report download disabled. Cap input file size to prevent unbounded memory use. | Unit/mock-DOM + exact-head |
| BL-DATA-004 | Change input after calculation | Invalidate the prior report and disable report download until the current input set is recalculated. An old report must not be presented as matching edited inputs. | Mock-DOM + exact-head |
| BL-DATA-005 | Download report | Export input assumptions, the deterministic economics report, the 12-month cash plan and its projection in a schema-version-2 envelope. The report file is an owner-controlled local artifact, not cloud backup or proof of compliance. | Mock-DOM + exact-head |
| BL-DATA-006 | Import actuals source | Accept only actuals-source schema v1 and supported cash-source types; require matching currency/precision, full calendar-month coverage within the dated forecast horizon, unique transaction IDs, valid in-range dates, a supported category, positive integer minor-unit amounts and source-row references. Require source opening/closing cash balances to reconcile exactly to imported net transactions. Compute SHA-256 from original imported bytes. Failure must not enable variance export or change the existing forecast report. | Core + mock-DOM + exact-head |
| BL-DATA-007 | Export variance evidence bundle | Export forecast start month, scenario input, cash plan/projection, comparison, exact imported source text and SHA-256 digest. Re-hashing the embedded UTF-8 source text must reproduce the digest. Editing scenario inputs invalidates the comparison/export until actuals are compared again. The report labels source authenticity as unverified. | Mock-DOM + Chromium + exact-head |

## 5. Suggested evidence corpus

Create small, deterministic fixtures with synthetic data:

1. **Service microbusiness:** one owner, two services, variable/fixed costs, quote → accepted job → completion → invoice evidence → reconciliation.
2. **Food microbusiness:** ingredients and perishable stock, food handling prerequisite missing/verified/expired, recipe costing, sale and waste. No real food-safety certification implied by fixtures.
3. **Retail shop:** SKU and stock movements, purchase receipt, sale, returns, stock reconciliation, and duplicate sale prevention.
4. **Provider failure:** accepted request then network timeout, duplicate webhook, reversed webhook order, stale status API, and a final authoritative settled/failed result.
5. **Multi-agent commerce:** buyer, seller and adversarial peer; conflicting revisions; unauthorized transition; cancellation vs shipment conflict; revocation; recovery after restart.
6. **Jurisdiction changes:** source expiration, rule effective-date change, municipal applicability mismatch, and missing official source.
7. **Privacy isolation:** two business tenants, authorized accountant, revoked operator, public listing opt-in, and export/delete request.
8. **Model uncertainty:** sparse sales history, distribution shift, missing costs, spurious outlier and a case where the correct response is abstention.

All fixtures should carry schema version, scenario ID, explicit expected state, frozen timestamps/timezone, fixed currency/rounding, and content digest. Keep real personal, bank, and merchant data out of the default corpus.

## 6. Required engineering workflow

Run this sequence whenever an implementation change claims to satisfy criteria:

1. **Automated deterministic simulation:** pure-function/property tests and scenario tests cover the declared happy path plus negative/fault paths.
2. **Capture review:** freeze expected outputs, transaction receipts, state traces, authorization decisions, export files, model inputs/outputs, and artifact digests; run an independent checker against those captures.
3. **Exact-head build and integration:** test the actual code revision and required external components using a pinned toolchain. Label unrun conductor scenarios as unrun.
4. **Owner/operator playthrough:** review the actual captured user journey for comprehensibility and missing steps. This tests product usability, not correctness of cryptographic, authorization or payment claims.
5. **Opt-in pilot:** only after legal/source review and safety gates; use measured real outcomes to evaluate usefulness, not to waive deterministic failures.

## 7. v0.1 release gates

The release remains **not releasable** until all are true:

- [ ] The owning repository/application boundary and license obligations are resolved.
- [ ] The selected pilot jurisdiction and activity have a source-reviewed prerequisite pack with effective dates and explicit unsupported cases.
- [ ] BL-SET-001 through BL-SET-008, BL-ECO-001 through BL-ECO-013, BL-COM-001 through BL-COM-008, BL-FIN-001 through BL-FIN-010, BL-DATA-001 through BL-DATA-007, and BL-SEC-001 through BL-SEC-009 pass at their required evidence levels for the implemented scope. BL-FIN-010 can pass v0.1 by proving the unapproved settlement path remains disabled.
- [ ] Marketplace contracts are preserved or a reviewed migration documents every changed invariant.
- [ ] Every consequential effect is capability-bound, payload-bound, idempotent where possible, and recoverable through an explicit failure/indeterminate path.
- [ ] No real-money movement, lending, stored value, or automated filing is present in v0.1.
- [ ] Export/import and backup restore are demonstrated with frozen fixtures.
- [ ] No unresolved critical/high authorization, tenant-isolation, privacy, or duplicate-effect failure remains.
- [ ] Public copy describes only the specific capabilities and evidence level achieved.
- [ ] The tested commit, toolchain, dependencies, source pack versions, and artifacts are recorded.

The unchecked boxes above are the target contract, not a report that work has been done or tests have passed.
