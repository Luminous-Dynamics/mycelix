# mycelix-business-profile-hospitality

A brand-agnostic hospitality/food-service **shadow profile** for the Mycelix Business Fabric.

This crate is a preset and qualification contract, not a restaurant ERP. It does not own sales, inventory, recipes, suppliers, workforce, Finance, Commerce, or authority state. It names the evidence and read-only intelligence capabilities a food-service operation can expose to the generic Business Shadow harness.

The profile is intentionally vendor-neutral and brand-neutral. A real franchise or independent restaurant supplies adapters and mappings outside this crate; the profile contains no POS-provider enum, franchise brand, country, currency, tax rule, bank rail, or write-capable API.

## Initial shadow lanes

- demand forecasting;
- prep recommendation;
- replenishment recommendation;
- waste-review recommendation.

Demand forecasting is qualified against an explicit preregistered baseline. Recommendation lanes are qualified through preregistered human-review protocols. Neither path claims hypothetical savings or causal impact from actions that were never executed.

The profile's maximum surface is A2 (`recommend`). Any later drafting or bounded execution must graduate into the Action Contract / authority / coordination / revalidation / receipt / reconciliation stack and earn separate field evidence.
