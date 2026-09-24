# FIN-SYS source profile v1

Status: design contract only; no ingestion authority.

## Purpose

Define the first read-only macro-financial evidence profile consumed by FIN-SYS-001 without pre-labelling any economy, sector, institution, or financing arrangement as a Ponzi scheme, fraud, or unsustainable system.

## Source families

Initial authoritative-in-own-domain sources:

- IMF Global Debt Database / Fiscal Monitor;
- BIS debt-service-ratio and credit datasets;
- World Bank International Debt Statistics / International Debt Report;
- later country central-bank, national-accounts, financial-accounts, or flow-of-funds sources under explicit profiles.

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
- release/version/publication date where available;
- geography;
- sector/population;
- period represented;
- retrieved/known-at time;
- unit, denominator, price/current-constant basis, and currency where applicable;
- transformation/calculation lineage;
- source artifact/digest where feasible;
- coverage/missingness;
- revision/supersession relationship;
- upstream provenance family.

Ten publications reusing one BIS/IMF series remain one upstream provenance family.

## Acquisition profiles

A source adapter MUST identify the exact acquisition mechanism it used. API access, bulk files, dashboards, and republished tables are not silently interchangeable evidence artifacts.

### BIS

The BIS currently exposes public statistics through an SDMX REST API whose v2 interface returns SDMX 2.1 data, and also publishes dated bulk downloads in CSV and SDMX forms.

The adapter profile MUST bind at least:

- API/bulk acquisition mode;
- API profile/version when API access is used;
- dataflow/resource/series identity and relevant structure metadata;
- release/publication date;
- requested/returned format;
- source artifact digest for retained bulk/response bytes where permitted;
- BIS methodology/coverage identity;
- applicable source terms/permitted-use profile.

Where a dated bulk release exists, retaining the exact admitted artifact is preferred for replayability; API responses still require a retrieval-time + response digest receipt.

### IMF

IMF Data currently exposes SDMX 2.1 and SDMX 3.0 APIs. An adapter MUST freeze which API/version/dataflow/structure it consumes rather than treating `IMF API` as one timeless interface.

The adapter MUST preserve dataset/dataflow identity, structure/codelist identity where relevant, period, release/publication metadata when available, and the exact transformation from source dimensions into normalized Mycelix dimensions.

### World Bank IDS

World Bank International Debt Statistics exposes API, DataBank, bulk files, and archived database releases. The adapter MUST bind the exact access path and dataset release.

IDS is an external-debt dataset with annual country/borrower-group scope under its reporting methodology; it MUST NOT be silently promoted into a complete domestic financial-accounts view.

Where licensing metadata is relied on for redistribution or retained fixtures, bind the license/profile observed for that release rather than assuming licensing is timeless. The current IDS catalog identifies the dataset as public and licensed CC BY 4.0.

## Initial semantic coordinates

Where supported by the exact source, preserve separately:

- debt stock;
- income/output denominator;
- debt-service ratio;
- interest burden;
- principal amortization;
- gross refinancing requirement;
- maturity distribution;
- fixed/floating-rate exposure;
- domestic/foreign-currency exposure;
- external debt service / net transfers;
- credit growth;
- productive investment / capital formation;
- asset-price/collateral context;
- banking/NBFI/sovereign dependency observations.

Missing coordinates remain `Unknown`. Do not impute required regime-classification coordinates merely to force classification.

## BIS DSR profile

BIS DSR is treated as a source-defined estimate of debt-service costs (interest plus amortization) relative to income for the covered private non-financial sectors/economies.

The adapter MUST preserve BIS methodology/coverage identity and MUST NOT reinterpret DSR as:

```text
probability of crisis
insolvency
fraud
Ponzi-finance status
```

Those are downstream analytical propositions.

The current BIS methodology notes an important comparability caveat: the internationally harmonized aggregate method is useful for tracing DSR changes through time, while absolute level comparisons across countries can be less reliable because of institutional, behavioral, maturity, and approximation differences.

Therefore:

```text
higher DSR level in country A than country B
!= stronger cross-country distress claim by itself
```

Cross-country comparisons MUST carry the source-methodology limitation rather than presenting the raw ranking as a directly comparable distress scale.

## IMF debt profile

Global/sector debt-to-GDP observations are debt-stock coordinates. They MUST NOT alone establish servicing stress or financing regime.

## World Bank external-debt profile

External debt-service, interest, principal, and related country/borrower-group observations remain external-debt coordinates under their exact reporting scope. They MUST NOT silently become complete public/private domestic-debt views.

Archived releases SHOULD be used when reconstructing `known at T` states where available rather than back-projecting today's revised series into historical information sets.

## Revision semantics

A later revised series creates successor evidence. Historical releases and the fact that an earlier release existed remain reconstructible when the source/archive permits it.

```text
latest release != only historical evidence
revised value != prior publication never existed
```

An adapter receipt distinguishes at least:

```text
ObservedSourceRelease
SupersededSourceRelease
CurrentSourceRelease
```

without implying that `CurrentSourceRelease` is factually final.

## Source/model airlock

The canonical pipeline is:

```text
source artifact
-> source-specific parser
-> normalized observation + receipt
-> derived metric under named transform
-> Symthaea analytical profile
-> scenario model
```

No stage may silently collapse into the next.

## Reproducibility and source drift

For every source family, qualification must cover both content and interface drift.

At minimum detect:

- API version/profile change;
- dataflow/series identifier change;
- dimension/codelist change;
- unit/denominator change;
- revised historical values;
- discontinued series;
- altered release cadence;
- changed methodology/coverage note;
- changed source-license/permitted-use profile where relevant.

A parser that still returns numbers after a source change has not necessarily remained semantically correct.

## Qualification requirements

Before automated refreshes, freeze fixtures for:

1. exact valid source row -> expected normalized observation;
2. missing/suppressed value;
3. revised historical value;
4. source-unit change;
5. geography/sector mismatch;
6. duplicate publication sharing the same upstream series;
7. transformation-profile change;
8. unsupported/malformed source payload;
9. stale cached release;
10. two sources disagreeing under non-equivalent methodology;
11. API/structure version drift;
12. changed methodology or coverage note;
13. historical archived release vs current revised release;
14. source-license/permitted-use metadata change.

## Nonclaims

This document establishes no live API adapter, currentness guarantee, dataset-license theorem beyond the exact recorded source metadata, statistical comparability across every source, causal inference, crisis forecast, policy recommendation, investment advice, economic-system ranking, fraud finding, or autonomous Symthaea authority.
