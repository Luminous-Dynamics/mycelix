# Mycelix Institution Console

The first shared Leptos professional-finance frontend surface.

One application shell serves multiple institution profiles rather than separate source forks.

Supported profiles:
- Bank
- Credit union / cooperative
- Payment institution / fintech
- Corporate treasury
- Custodian / market infrastructure
- Commons / cooperative finance

The current home screen uses synthetic values and is not connected to live financial state.

Production integration begins with:
1. Reconciliation
2. Treasury/liquidity
3. Settlement/evidence

Security rule: browser state is advisory. All material authorization must be enforced outside the browser with explicit principal, tenant, legal entity, capability and policy context.

The implementation uses stable Leptos 0.8.19 and Axum 0.8, following the official Leptos Axum starter topology.
