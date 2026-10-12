# Business Launchpad Browser Prototype

This is a small static front end for the deterministic viability worksheet. It is deliberately not a full business application: it has no sign-in, database, external API, payment flow, POS, accounting ledger, tax rules, legal registration, Symthaea integration or Mycelix network integration.

## Run locally

From the Mycelix repository root:

    cd mycelix-workspace/business-launchpad
    python3 -m http.server 8080 --bind 127.0.0.1

Then open http://127.0.0.1:8080/web/ in your browser. Bind to loopback so the development server is not exposed to the local network.

No package install is required. The page imports the dependency-free calculator in `../core/src/index.js`. Calculations run in browser memory; the application does not upload inputs or use browser storage. Use **Download assumptions JSON** to save a portable versioned input file, **Load assumptions JSON** to restore one, and **Download calculated report JSON** after calculating to save the report alongside its inputs. These are explicit local file operations, not cloud backup. If you change any input, the prior report is invalidated until you calculate again. The static assets are still fetched from the local web server.

## Important boundaries

- Initial numbers are fictional examples; replace them with verified quotes and realistic assumptions.
- It uses a single-offer, one-month arithmetic model, not a proper multi-period cash-flow forecast.
- It does not fetch or verify currency support, tax thresholds, licenses or municipal requirements.
- “Complete costs” is only the owner's declaration for the entered scope, not an independent audit.
- A positive pre-exclusion result does not establish profitability, demand, compliance, or a safe decision to invest.

Accessibility basics include semantic landmarks, labeled controls, keyboard focus indicators, readable contrast, an alert for invalid inputs, and a live report region. Browser accessibility and mobile usability have not yet been independently audited.
