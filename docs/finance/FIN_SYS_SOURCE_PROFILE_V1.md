# FIN-SYS source profile v1

Status: provisional design contract only; no ingestion authority. This file was staged directly on `main` by the current connector path and requires ordinary repository review before being treated as accepted architecture.

## Purpose

Define the first read-only macro-financial evidence profile consumed by FIN-SYS-001 without pre-labelling any economy, sector, institution, or financing arrangement as a Ponzi scheme, fraud, or unsustainable system.

## Source families

Initial authoritative-in-own-domain sources:

- IMF Global Debt Database / Fiscal Monitor;
- BIS debt service ratio and credit datasets;
- World Bank International Debt Statistics / International Debt Report;
- later country central-bank / national-accounts / flow-of-funds sources under explicit profiles.

External institutional publication is source evidence, not Mycelix truth.

## Core distinctions

```text
high debt != high debt-service burden
high debt-service burden != refinancing dependence
refinancing dependence != Minsky Ponzi finance
Minsky Ponzi finance != criminal Ponzi fraud
credit growth != productive investment
asset-price dependence != manipulation
provider revision != historical observation never existed
```

## Observation identity

Every imported observation MUST bind:

- provider/source family;
- dataset/table/series identifier;
- release/version/publication date;
- geography;
- sector/population;
- period represented;
- retrieved/known-at time;
- unit, denominator, price/current-constant basis and currency where applicable;
- transformation/calculation lineage;
- source artifact/digest where feasible;
- coverage/missingness;
- revision/supersession relationship;
- upstream provenance family.

Ten publications reusing one BIS/IMF series remain one upstream provenance family.

## Initial semantic coordinates

Where supported by the exact source, preserve separately:

- debt stock;
- income/output denominator;
- debt-service ratio;
- interest burden;
- principal amortization;
- gross refinancing requirement;
- maturity distribution;
- fixed/floating rate exposure;
- domestic/foreign currency exposure;
- external debt service / net transfers;
- credit growth;
- productive investment / capital formation;
- asset-price/collateral context;
- banking/NBFI/sovereign dependency observations.

Missing coordinates remain `Unknown`. Do not impute required regime-classification coordinates merely to force classification.

## BIS DSR profile

BIS DSR is treated as a source-defined estimate of debt-service costs (interest + amortisation) relative to income for the covered private non-financial sectors/economies.

The adapter MUST preserve BIS methodology/coverage identity and MUST NOT reinterpret DSR as:

```text
probability of crisis
insolvency
fraud
Ponzi-finance status
```

Those are downstream analytical propositions.

## IMF debt profile

Global/sector debt-to-GDP observations are debt-stock coordinates. They MUST NOT alone establish servicing stress or financing regime.

## World Bank external-debt profile

External debt-service, interest, principal and related country/borrower-group observations remain external-debt coordinates under their exact reporting scope. They MUST NOT silently become complete public/private domestic debt views.

## Revision semantics

A later revised series creates successor evidence. Historical bytes and the fact that an earlier release existed remain reconstructible.

```text
latest release != only historical truth record
```

## Nonclaims

This document establishes no live API adapter, currentness guarantee, dataset-license theorem, statistical comparability across every source, causal inference, crisis forecast, policy recommendation, or economic-system verdict.
