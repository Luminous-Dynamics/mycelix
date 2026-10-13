# Business Launchpad Core — first executable slice

**Status:** Experimental local calculation module; not a full business application.

This dependency-free Node.js module computes a transparent single-offer scenario and a separate bounded month-by-month cash-liquidity scenario. It is UI-, database-, payment-, and Holochain-agnostic so a business-facing interface can consume the domain contract without putting merchant business logic in the NixOS runtime or Mycelix Finance tokenomics.

## What it computes today

- Unit contribution = selling price − variable cost.
- Monthly scenario revenue, variable cost, contribution, and operating result before excluded items.
- Break-even units using integer ceiling arithmetic. If unit contribution is zero or negative, break-even is explicitly unreachable.
- Startup cash requirement = one-time setup costs + an explicit working-capital buffer.
- Startup funding gap after available starting cash; this compares setup costs plus the owner-entered buffer with available startup cash, not the full cash runway needed through a slow sales ramp.
- Explicit cost-coverage state and named omitted categories.
- A separate 1–24 month cash-runway scenario with independent month-by-month sales activity, sales cash receipts, operating cash payments and other inflows/outflows; it reports post-setup cash, lowest projected cash, first reserve breach, first negative balance, and the additional buffer needed to maintain the reserve.
- `projectCashRunwayWithTimingSchedule` builds a separate terms-based alternative using an explicit current/+1/+2 month collection schedule and three variable-cost categories. Their cost shares total 100%; each category has its own payment distribution totaling 100%. It derives gross receipts from units × price, uses integer minor-unit apportionment to split the entered unit cost without changing the total, leaves fixed overhead in its originating month, carries other/investing/financing movements unchanged, and discloses receipts and each category's payments beyond the horizon. It does not overwrite the entered base plan or model invoice-level receivables/payables.
- `compareCashRunwayScenarios`: a three-column downside/base/upside comparison using the owner-entered base plan and explicit configurable stress assumptions. The downside reduces receipts, delays collection and increases operating payments; the upside increases receipts, advances later receipts within the horizon and reduces operating payments. The exact assumptions and full projection for every case are included in exports; receipts delayed past the horizon are disclosed separately.

All amounts use integer minor units for the chosen currency. The implementation uses BigInt intermediates and fails closed when any returned result would not fit exactly in JavaScript's safe-integer range. Stable amount formatting is provided for tests and evidence; localized formatting belongs in the UI.


## Auditable actuals-versus-plan comparison

`compareCashPlanToActuals` compares the existing cash plan with a separate schema-v1 actuals-source JSON file. The source object includes a supported `type` (`bank_statement_export` or `reconciled_cash_ledger_export`), a descriptive label/reference, full-month coverage dates, an explicit completeness declaration, and source opening/closing cash balances. Each transaction needs a unique ID, date, matching currency code, one supported cash-flow category, a positive integer `amountMinor`, and a source row/statement reference.

Set `forecastStartMonth` to the calendar month represented by forecast Month 1. Coverage must consist of complete calendar months within the forecast horizon. The engine fails closed on invalid/duplicate IDs, missing references, currency mismatch, out-of-coverage dates, unsupported categories, overflow, incomplete coverage or a difference between imported net transactions and the source opening-to-closing cash movement. Variances are reported as actual minus plan per cash bucket and for net cash movement; closing-balance variance also reflects any opening-balance difference.

The static client hashes the exact imported UTF-8 JSON bytes with SHA-256 and exports the original decoded source text, digest, scenario inputs, cash plan, projection and comparison in a separate evidence bundle. An independent verifier can re-hash the embedded source text and rerun the deterministic comparison. The fingerprint proves byte identity relative to the bundle, **not** bank/ledger authenticity or completeness. Intermediate monthly closing actuals are derived from transactions; only the ending cash balance for the full imported range is separately declared and reconciled. This is cash-basis sensitivity analysis, not accrual accounting, tax reporting, profit recognition, an invoice ledger or proof of compliance.

The imported schema is intentionally normalized around cash records. Keep original statements, receipts and ledger exports in the user's secure record system; `evidenceReference` fields identify supporting records but do not embed or independently validate them.

## Run

Requires Node.js 24.21.0. No dependency installation is required.

    npm run check
    npm test
    npm run example

To analyze your own JSON scenario:

    npm run analyze -- path/to/scenario.json

The CLI prints a JSON report on success. Invalid scenario data or an unreadable file exits non-zero and does not print a report. It reads only the file explicitly passed; it does not connect to other services.

The bundled service example is invented arithmetic test data, not a researched local price benchmark or recommendation.

## Input example

    {
      currency: { code: 'ZAR', minorUnitExponent: 2 },
      sellingPricePerUnitMinor: 10000,
      variableCostPerUnitMinor: 6000,
      fixedMonthlyCostsMinor: 100000,
      plannedUnitsPerMonth: 300,
      setupCostsMinor: 400000,
      initialWorkingCapitalBufferMinor: 100000,
      availableCashForStartupMinor: 300000,
      costCoverage: {
        level: 'partial',
        omittedCategories: ['owner pay', 'insurance']
      }
    }

## Hard boundaries

Currency-code shape validation is not authoritative currency validation. The core does not determine tax treatment, legal structure, licenses, or demand. The monthly operating result from `analyzeBusinessModel` is not net profit: it excludes tax, financing/debt service, depreciation, owner draw, and any cost not included in the supplied cost model. Each unit means one average-priced offer/sale defined by the owner; neither function models a product/service mix, inventory-specific costing, capacity constraints, or multiple offers. `projectCashRunway` is a deterministic scenario rather than a demand forecast. Sales units are an activity assumption; sales receipts and operating cash payments are separately entered by month, so delayed collection and advance payments can be represented. Older version-1 cash plans missing these fields receive same-month defaults for compatibility; new browser defaults also begin with same-month-derived values, which users should override when timing differs. This is not an accounts-receivable/payable ledger and does not reconcile invoices, settlement or supplier balances. Taxes, debt service, owner draw and other unmodeled outflows can be entered in the month-specific other-outflow field where known. Its reserve gap is not a loan offer or underwriting decision.

No customer data, payment processing, credit issuance, or legal filing is part of this core.

## Deliberately out of scope

- Accounting ledger, tax engine, payroll, POS, inventory/perishability, and multi-product optimization.
- Payment providers, banking integrations, credit issuance, or stored-value balances.
- Regulatory conclusions and automatic filings.
- AI-generated numbers or forecasts.
- Trust/reputation-based credit eligibility.

A static browser worksheet in `../web/` imports this core and provides the one-month economics report plus a separate editable 12-month cash scenario. New scenario and report downloads use schema version 6 so older clients reject the richer cash-flow, stress and category-specific timing contract rather than silently discard it. Schema 6 requires monthly sales-receipt/operating-payment/investing/financing fields, explicit downside/upside stress assumptions, one customer-collection allocation, and three named variable-cost categories. Cost shares total exactly 100%, and each category's three payment buckets also total exactly 100%. The importer accepts schema versions 1–5 through explicit migrations: version 1 may derive missing receipt/payment timing from same-month economics; version 2 preserves explicit timing and defaults new investing/financing buckets to zero; version 3 preserves cash categories and receives illustrative stress defaults; version 4 preserves stress settings and receives illustrative timing defaults; version 5's single supplier-payment curve is applied to all three categories, preserving its aggregate timing. Legacy version-1 scenarios without a plan receive an illustrative ramp; versions 2–6 require the 12-month plan, and versions 4–6 must include stress assumptions. The older generic `otherCashInflowsMinor`/`otherCashOutflowsMinor` fields remain an unclassified residual: they continue to affect total receipts/payments and closing cash but are not guessed into operating, investing or financing subtotals. Inputs remain in memory, prior results are invalidated when inputs change, and data is not uploaded or stored in browser storage. These local-file workflows are not cloud backup or durable business records. The mock-DOM test contract covers file round-trips, invalidation, malformed/oversized imports, and cash-runway rendering; all new changes still require exact-head CI execution. The Chromium journey in `test/browser-journey.mjs` is the real-engine test, but its run must finish and its artifact be reviewed before browser behavior is qualified. Its pinned Playwright dependency is isolated under `test/` and does not add a runtime dependency to the calculator. A persistence/integration layer should follow only after the current exact-head gates are reviewed and the source of truth for orders and financial records is selected. Jurisdiction-specific guidance must live in separately reviewed, versioned source packs.
