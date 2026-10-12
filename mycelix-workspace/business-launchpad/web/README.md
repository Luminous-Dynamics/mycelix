# Business Launchpad Browser Prototype

This is a small static front end for the deterministic viability worksheet. It is deliberately not a full business application: it has no sign-in, database, external API, payment flow, POS, accounting ledger, tax rules, legal registration, Symthaea integration or Mycelix network integration.

## Run locally

From the Mycelix repository root:

    cd mycelix-workspace/business-launchpad
    python3 -m http.server 8080 --bind 127.0.0.1

Then open http://127.0.0.1:8080/web/ in your browser. Bind to loopback so the development server is not exposed to the local network.

No package install is required. The page imports the dependency-free calculator in `../core/src/index.js`. Calculations run in browser memory; the application does not upload inputs or use browser storage. Use **Download assumptions JSON** to save a portable schema-v5 input file, **Load assumptions JSON** to restore one, and **Download calculated report JSON** after calculating to save the inputs, stress assumptions, terms-based collection/payment schedule, and all projections. Schema 5 captures operating/investing/financing cash movements, explicit downside/upside assumptions, and two three-bucket schedules. Each schedule's current/+1/+2 month percentages must total 100. The terms-based case is generated separately from the owner-entered base plan: gross sales are allocated by collection terms; variable costs are allocated by supplier terms; fixed overhead remains same-month. Schema 1–4 files are supported via explicit migration; schema 4 retains its stress settings and receives illustrative terms-schedule defaults. Versions 2–5 require a 12-month plan; versions 4–5 require stress assumptions and version 5 requires timing schedules. The legacy generic “other” cash fields remain unclassified during import: they continue to change total cash but are excluded from operating/investing/financing subtotals until the user reclassifies the transaction. This avoids inventing category semantics when migrating old files. These are explicit local file operations, not cloud backup. If you change any input, the prior report is invalidated until you calculate again. The static assets are still fetched from the local web server.

The exact-head Chromium journey is maintained at `../core/test/browser-journey.mjs` and run by `.github/workflows/business-launchpad-browser.yml`. It covers a narrow single-browser path and captured evidence; passing it would not certify screen-reader compatibility or full WCAG conformance.

## Important boundaries

- Initial numbers are fictional examples; replace them with verified quotes and realistic assumptions.
- The unit-economics worksheet uses one average-priced offer and one month of arithmetic; define one unit consistently (e.g., one cleaning job, one meal, or one retail item).
- The separate 12-month cash scenario lets you edit sales activity, sales cash receipts, operating cash payments and other inflows/outflows independently month by month. It deducts setup costs before Month 1 and treats the working-capital buffer as the reserve floor. The result also compares downside/base/upside cases and a separate terms-based timing case. The base remains the exact entered plan; the stress cases transform only sales receipts and operating cash outflows using the six visible/editable assumptions. The terms-based case derives gross receipts from the sales-unit plan and applies the editable current/+1/+2 month collection and variable-cost payment distributions (each sums to 100%), with fixed monthly costs remaining due in their originating month. Per-unit variable costs are treated as one payable cohort; this is not an invoice-level ledger and may need more categories for businesses with different variable-cost terms. Defaults (20% lower receipts with a one-month delay and 15% higher operating outflows; upside 10% higher receipts, one-month acceleration of later receipts and 5% lower operating outflows) are illustrative, not benchmarks or probabilities. Receipts shifted past Month 12 are listed separately, not counted as collected.
- Starter receipts/payments are generated from the single-offer worksheet as a convenience only; edit them to reflect delayed customer collection or payments made before sales cash arrives. Generated operating outflows combine base variable costs and fixed monthly costs; when editing them, enter the total operating cash payment for that month and place only additional costs in other outflows. Do not count the same expense twice. The **Reset cash plan to illustrative defaults** button regenerates the full 12-month plan and overwrites manual month-by-month edits. The prototype does not maintain or reconcile actual receivable/payable balances, invoices, provider settlement or supplier credit. Capacity, inventory-specific costing, seasonality and demand forecasting are not modeled. Add tax, owner draw, debt service and other known excluded outflows to the month-specific other-cash-outflow fields.
- It does not fetch or verify currency support, tax thresholds, licenses or municipal requirements.
- “Complete costs” is only the owner's declaration for the entered scope, not an independent audit.
- A positive pre-exclusion result does not establish profitability, demand, compliance, or a safe decision to invest.

Accessibility basics include semantic landmarks, labeled controls, keyboard focus indicators, readable contrast, an alert for invalid inputs, and a live report region. Browser accessibility and mobile usability have not yet been independently audited.
