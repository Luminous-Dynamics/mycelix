# Business Launchpad Core — first executable slice

**Status:** Experimental local calculation module; not a full business application.

This dependency-free Node.js module computes a transparent single-offer scenario. It is UI-, database-, payment-, and Holochain-agnostic so a future business-facing interface can consume the domain contract without putting merchant business logic in the NixOS runtime or Mycelix Finance tokenomics.

## What it computes today

- Unit contribution = selling price − variable cost.
- Monthly scenario revenue, variable cost, contribution, and operating result before excluded items.
- Break-even units using integer ceiling arithmetic. If unit contribution is zero or negative, break-even is explicitly unreachable.
- Startup cash requirement = one-time setup costs + an explicit working-capital buffer.
- Startup funding gap after available starting cash.
- Explicit cost-coverage state and named omitted categories.

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

Currency-code shape validation is not authoritative currency validation. The core does not determine tax treatment, legal structure, licenses, or demand. The monthly operating result is not net profit: it excludes tax, financing/debt service, depreciation, owner draw, and any cost not included in the supplied cost model. Sales volume is an owner-supplied scenario, not a Symthaea prediction. The funding gap is not a loan offer or underwriting decision.

No customer data, payment processing, credit issuance, or legal filing is part of this core.

## Deliberately out of scope

- Accounting ledger, tax engine, payroll, POS, inventory/perishability, and multi-product optimization.
- Payment providers, banking integrations, credit issuance, or stored-value balances.
- Regulatory conclusions and automatic filings.
- AI-generated numbers or forecasts.
- Trust/reputation-based credit eligibility.

A static browser worksheet now imports this core from `../web/`; it is a local prototype, not a hosted business application. Next, add automated browser interaction checks and review the deterministic output capture, then build a separate persistence/integration layer only after the source of truth for orders and financial records is selected. Jurisdiction-specific guidance must be held in separately reviewed, versioned source packs.
