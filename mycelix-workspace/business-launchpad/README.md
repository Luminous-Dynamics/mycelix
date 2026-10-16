# Business Launchpad — experimental slice

This directory contains the first small, executable part of the proposed Business Launchpad. It is **not a production business-management system**.

## Run the worksheet

From this directory:

    python3 -m http.server 8080 --bind 127.0.0.1

Open http://127.0.0.1:8080/web/ in a browser. Bind to loopback so the static development server is not exposed to your local network.

The browser worksheet calculates in page memory. It does not upload entered figures or store them in browser storage. Use **Download assumptions JSON** to save a schema-v6 scenario file, **Load assumptions JSON** to restore it, and **Download calculated report JSON** to save input assumptions, cash projections, stress assumptions and the terms-based collection/payment case. Schema 6 adds category-specific variable-cost timing: the existing per-sale cost is split across three named categories, their cost shares total 100%, and each category's payment schedule totals 100%. The terms-based case remains separate from the manually entered base plan. Version-1 through version-5 files remain importable with explicit migration rules; schema-v5's single supplier-payment curve is applied uniformly to all categories. Versions 2–6 require a monthly plan. Generic “other” cash values stay unclassified after migration and remain included in total cash; the model does not silently recast them as operating or financing transactions. Explicit schema versions prevent older clients from silently dropping richer data. A file saved on disk is not an automatic backup or business record service.

The **Actuals versus plan** panel adds a separate local actuals-import workflow. It requires a date mapping for forecast Month 1, full-month coverage, distinct transaction IDs, source-row references, matching currency and exact opening/closing-balance reconciliation. The exported variance evidence bundle contains the imported schema-v1 source text and SHA-256 digest alongside the selected scenario, plan, projection, monthly cash-bucket variances and balance comparisons. The digest can be recomputed; it does not authenticate a bank or prove completeness. Intermediate monthly closing balances are derived from imported rows. No bank or accounting API connection exists.

## Run the deterministic calculator tests

Requires Node.js 24.21.0, as pinned by the branch's GitHub Actions workflow. No npm dependencies are required.

    cd core
    npm run check
    npm test
    npm run example

For a custom scenario JSON file:

    npm run analyze -- path/to/scenario.json

The CLI reads only the path you provide and emits a JSON report, or returns a non-zero exit code on invalid input.

## What is implemented

- Single-offer unit contribution and one-month scenario arithmetic; define one unit consistently (for example, one cleaning job, one meal, or one retail item).
- Upward-rounded break-even volume, or explicit unreachable status when contribution is not positive.
- Setup cash + owner-entered working-capital buffer and the upfront funding gap against available startup cash.
- A separate editable 12-month cash scenario with sales units, sales cash receipts, operating cash outflows and other/unclassified/investing/financing movements entered independently by month. It shows closing balances, reserve breaches, first negative balance, lowest cash and extra buffer requirements.
- A separate terms-based scenario allocates gross collections using an editable three-bucket schedule and splits variable cost into three named categories. Category shares must total 100%, and each category has its own three-bucket payment schedule that also totals 100%. Fixed overhead stays due in its originating month. Cash landing past Month 12 is disclosed. This remains a timing sensitivity, not a due-date ledger, and it does not overwrite the entered base case. Reset regenerates the 12-month base plan and overwrites cash-plan edits.
- A deterministic downside/base/upside cash comparison with editable stress settings. It shows lowest and ending cash, reserve-breach/negative-balance timing, extra buffer needs and receipts delayed past the selected horizon. These are explicit sensitivity cases, not probability-weighted forecasts.
- Exact decimal-text parsing into integer minor units; no binary floating-point amount rounding.
- Explicit partial-cost declaration and named omissions.
- Synthetic service, food and retail test fixtures.
- Static browser inputs/results and portable scenario/report JSON export/import.

For the separate Chromium journey and its isolated locked test dependency, see [`core/README.md`](core/README.md). The browser workflow is the canonical exact-head run; do not treat a local smoke run or source-level accessibility guard as full browser/accessibility qualification.

## What is not implemented

There is no account system, durable database, multi-user tenancy, full general ledger, inventory/POS, payment processor, external accounting integration, tax or regulatory rule engine, Symthaea forecasting integration, or Mycelix Marketplace/Finance integration. The calculations are not tax/legal advice, a net-profit statement, a demand forecast, an underwriting decision, or a promise that a business will succeed.

Example inputs are fictional. The unit-economics calculation assumes one average-priced offer; the separate cash projection allows independently timed receipts/payments but does not reconcile real invoices, aged receivables/payables or supplier balances. Neither models offer mix, inventory-specific costing, capacity constraints, seasonality or demand. Replace example inputs with current supplier quotes, credible demand information, all applicable costs, and authoritative local requirements before using a result to make a financial commitment.
