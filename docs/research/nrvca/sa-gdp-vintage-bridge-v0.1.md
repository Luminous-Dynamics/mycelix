# NRVCA South Africa GDP release-vintage bridge v0.1

**Status:** discovery-only, unpinned manual transcription  
**Fixture:** [Table 1 transcription](../../../fixtures/nrvca-sa-gdp-table1-transcription-v0.1.json)  
**Rust checks:** `mycelix-workspace/tools/nrvca-validator/tests/sa_gdp_vintage_bridge.rs`  
**Tracking issue:** [NRVCA-011 / #5429](https://github.com/Luminous-Dynamics/mycelix/issues/5429)

## What was acquired—and what was not

As checked on **22 October 2026**, the official Stats SA P0441 publication page listed *Gross domestic product, quarter ended June 2026*, released **8 September 2026**, along with a downloadable PDF and companion time-series XLSX:

- Release page: https://www.statssa.gov.za/?PPN=P0441&page_id=1854
- Main PDF: https://www.statssa.gov.za/publications/P0441/P04412ndQuarter2026.pdf
- Companion time-series XLSX locator: https://www.statssa.gov.za/publications/P0441/GDP%20P0441%20-%20GDP%20Time%20series%20Q2%202026.xlsx
- Publication and Data Access Policy: https://www.statssa.gov.za/?page_id=18324

The browser opened the official PDF and Table 1 was visually inspected (printed page 8; annual and quarterly blocks). The PDF and XLSX *bytes were not acquired* by this work session: download attempts failed. Therefore the fixture records `source_bytes_acquired=false`, `source_sha256=null`, and `extraction_status=not_independently_reparsed_from_source_bytes`. No upstream digest or reproducible extraction claim is made. The JSON contains a transcription of displayed numbers; its own version control does not pin the publisher's PDF bytes.

Stats SA's release text says that the 2022-base rebased and benchmarked national-account estimates are expected in October 2026. The live official page checked on 22 October still described the release as expected; a separate rebase release was not located in this pass. This means **availability is unresolved**, not that the release definitively does not exist. The follow-up must recheck the authoritative source and add a separate vintage if it has appeared.

## What the fixture contains

The provisional Table 1 transcription contains:
- **8 annual rows** (2018–2025), displayed as annual totals;
- **12 quarterly rows** (2023 Q3–2026 Q2), displayed as seasonally adjusted and annualised rates;
- 13 numeric columns: ten industries, total value added at basic prices, taxes less subsidies, and GDP at market prices;
- units of R million at constant 2015 prices.

The annual and quarterly blocks are separate frequencies and have different adjustment conventions. They must not be concatenated as if one homogeneous time series. Annual 2025 GDP in this table is R4,716,601 million at constant 2015 prices. The 2026 Q2 quarterly seasonally adjusted and annualised GDP level is R4,754,800 million at constant 2015 prices. This is a narrow table transcription, not an NRVCA national outcome account.

## Reconciliation is not the same as forcing exact equality

The displayed values are rounded to integer millions. The row checks retain **the actual displayed residual** rather than changing reported data to make identities look exact:

- Sum of the ten displayed industry values minus displayed total value added: absolute tolerance **5 R million**.
- Displayed value added plus displayed taxes less subsidies minus displayed GDP: absolute tolerance **1 R million**.

Across this transcription, the largest absolute sector-sum residual is 3 R million; the largest absolute GDP bridge residual is 1 R million. These checks test arithmetic consistency with displayed rounding. They do **not** prove each transcribed source cell is correct or validate the underlying national accounts methodology; source-byte acquisition and comparison with the companion XLSX remain mandatory.

The Rust tests freeze identity/vintage/frequency boundaries, row counts, key reference values, residual computation, conservative tolerances, duplicate-period rejection and negative controls. They are intended as contract checks; they have not been reported as passed until executed on an exact head.

## Paradigm shift: immutable release-vintage observations

A data point is not just `GDP=...`. It is a value plus source publisher, release vintage, reference period, price base, frequency, seasonal-adjustment convention, units, source row, extraction method, source-byte status, and explicit reconciliation residuals.

A future 2022-base release must not overwrite 2015-base values silently. NRVCA should retain both series as separate immutable vintages, identify overlap periods, compare levels and growth rates, document revised classifications and benchmark changes, and publish a vintage bridge before any model or dashboard changes its input series.

This composes with official national accounts rather than attempting to replace them. The fixture is useful for prototype validator development but is not evidence of better measurement, better forecasts, causal policy impact or actual Johannesburg infrastructure capacity.

## Qualification gates

1. Acquire the PDF/XLSX bytes from the official endpoint, record SHA-256 and retrieval timestamp, retain source terms, and independently parse or transcribe them.
2. Compare all 260 displayed numeric cells with the companion XLSX and/or a second independent extraction. Resolve differences and freeze a known-answer file only after reconciliation.
3. Re-run accounting and frequency tests on the source-pinned extract, retaining rounding residuals instead of silently normalizing figures.
4. Recheck whether Stats SA released the 2022-base benchmark/rebase series. If available, ingest as a distinct vintage; compare overlap values and revisions without splicing across bases.
5. Add real water/energy service observations only with their own source identities, units, geography, cadence, licenses, missingness, and byte digests. GDP value added is not a proxy for service delivery.
6. Execute the focused Rust tests on an exact recorded commit. Then obtain independent review and only after that qualify the data pipeline.

### Evidence references

- Stats SA P0441 official release: https://www.statssa.gov.za/?PPN=P0441&page_id=1854
- Main release PDF, particularly Table 1 printed page 8 and its explanatory text: https://www.statssa.gov.za/publications/P0441/P04412ndQuarter2026.pdf
- Stats SA Publication and Data Access Policy: https://www.statssa.gov.za/?page_id=18324
- NRVCA's overall framework and source-vintage discipline: [NRVCA README](./README.md)

