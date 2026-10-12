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
- A separate 1–24 month cash-runway scenario with month-specific sales units, other cash inflows/outflows, post-setup cash, lowest projected cash, first reserve breach, first negative balance, and the additional buffer needed to maintain the reserve.

All amounts use integer minor units for the chosen currency. The implementation uses BigInt intermediates and fails closed when any returned result would not fit exactly in JavaScript's safe-integer range. Stable amount formatting is provided for tests and evidence; localized formatting belongs in the UI.

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

Currency-code shape validation is not authoritative currency validation. The core does not determine tax treatment, legal structure, licenses, or demand. The monthly operating result from `analyzeBusinessModel` is not net profit: it excludes tax, financing/debt service, depreciation, owner draw, and any cost not included in the supplied cost model. Each unit means one average-priced offer/sale defined by the owner; neither function models a product/service mix, inventory-specific costing, capacity constraints, or multiple offers. `projectCashRunway` is a deterministic scenario rather than a demand forecast: it assumes each listed sale is collected in full in the same month and its variable cost is paid in that month; receivables, delayed settlements, inventory paid in advance and supplier-credit timing are not modeled. Taxes, debt service, owner draw and other unmodeled outflows must be entered in the month-specific other-cash-outflow field where known. Its reserve gap is not a loan offer or underwriting decision.

No customer data, payment processing, credit issuance, or legal filing is part of this core.

## Deliberately out of scope

- Accounting ledger, tax engine, payroll, POS, inventory/perishability, and multi-product optimization.
- Payment providers, banking integrations, credit issuance, or stored-value balances.
- Regulatory conclusions and automatic filings.
- AI-generated numbers or forecasts.
- Trust/reputation-based credit eligibility.

A static browser worksheet in `../web/` imports this core and provides the one-month economics report plus a separate editable 12-month cash scenario. Versioned scenario JSON includes the month-by-month cash plan; the calculated report exports both arithmetic and cash projection. Inputs remain in memory, prior results are invalidated when inputs change, and data is not uploaded or stored in browser storage. These local-file workflows are not cloud backup or durable business records. The mock-DOM test contract covers file round-trips, invalidation, malformed/oversized imports, and cash-runway rendering; all new changes still require exact-head CI execution. The Chromium journey in `test/browser-journey.mjs` is the real-engine test, but its run must finish and its artifact be reviewed before browser behavior is qualified. Its pinned Playwright dependency is isolated under `test/` and does not add a runtime dependency to the calculator. A persistence/integration layer should follow only after the current exact-head gates are reviewed and the source of truth for orders and financial records is selected. Jurisdiction-specific guidance must live in separately reviewed, versioned source packs.
