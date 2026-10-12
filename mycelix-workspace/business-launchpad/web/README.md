# Business Launchpad Browser Prototype

This is a small static front end for the deterministic viability worksheet. It is deliberately not a full business application: it has no sign-in, database, external API, payment flow, POS, accounting ledger, tax rules, legal registration, Symthaea integration or Mycelix network integration.

## Run locally

From the Mycelix repository root:

    cd mycelix-workspace/business-launchpad
    python3 -m http.server 8080 --bind 127.0.0.1

Then open http://127.0.0.1:8080/web/ in your browser. Bind to loopback so the development server is not exposed to the local network.

No package install is required. The page imports the dependency-free calculator in `../core/src/index.js`. Calculations run in browser memory; the application does not upload inputs or use browser storage. Use **Download assumptions JSON** to save a portable schema-v2 input file, **Load assumptions JSON** to restore one, and **Download calculated report JSON** after calculating to save the report alongside its inputs. Version-1 scenario files remain importable through an explicit compatibility path; version-2 files require the 12-month cash plan so an older client cannot silently lose the newer timing fields. These are explicit local file operations, not cloud backup. If you change any input, the prior report is invalidated until you calculate again. The static assets are still fetched from the local web server.

The exact-head Chromium journey is maintained at `../core/test/browser-journey.mjs` and run by `.github/workflows/business-launchpad-browser.yml`. It covers a narrow single-browser path and captured evidence; passing it would not certify screen-reader compatibility or full WCAG conformance.

## Important boundaries

- Initial numbers are fictional examples; replace them with verified quotes and realistic assumptions.
- The unit-economics worksheet uses one average-priced offer and one month of arithmetic; define one unit consistently (e.g., one cleaning job, one meal, or one retail item).
- The separate 12-month cash scenario lets you edit sales activity, sales cash receipts, operating cash payments and other inflows/outflows independently month by month. It deducts setup costs before Month 1 and treats the working-capital buffer as the reserve floor.
- Starter receipts/payments are generated from the single-offer worksheet as a convenience only; edit them to reflect delayed customer collection or payments made before sales cash arrives. The **Reset cash plan to illustrative defaults** button regenerates the full 12-month plan and overwrites manual month-by-month edits. The prototype does not maintain or reconcile actual receivable/payable balances, invoices, provider settlement or supplier credit. Capacity, inventory-specific costing, seasonality and demand forecasting are not modeled. Add tax, owner draw, debt service and other known excluded outflows to the month-specific other-cash-outflow fields.
- It does not fetch or verify currency support, tax thresholds, licenses or municipal requirements.
- “Complete costs” is only the owner's declaration for the entered scope, not an independent audit.
- A positive pre-exclusion result does not establish profitability, demand, compliance, or a safe decision to invest.

Accessibility basics include semantic landmarks, labeled controls, keyboard focus indicators, readable contrast, an alert for invalid inputs, and a live report region. Browser accessibility and mobile usability have not yet been independently audited.
