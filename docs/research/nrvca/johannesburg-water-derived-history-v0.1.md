# NRVCA Johannesburg Water derived status-history admission v0.1

**Parent:** [Mycelix #6270 — NRVCA-012F](https://github.com/Luminous-Dynamics/mycelix/issues/6270)  
**Source register:** PR #6110 / first-party source register v0.1  
**Event graph:** PR #6111 / event graph v0.1  
**Evidence class:** third-party-derived operator report; metadata-only at this revision  
**Qualification:** NotRun

## Why this is a distinct source class

Watersearch's public read-only API describes a daily history compiled from Johannesburg Water reservoir/tower status boards and notices since 3 March 2025. Its pages explicitly state that Watersearch is not run by Johannesburg Water or the City. It is a compilation of operator-published reporting, not a separate observation system.

The API release identified on 15 November 2026 is `2026-11-15-b7c711ba`. The release endpoint's licence field says CC BY 4.0 for the compilation/readings and asks users to credit Johannesburg Water for the underlying facts. The overview still describes the licence as provisional until the site is public. The API and download pages were publicly accessible during this review, but the rights review remains open until acquisition/release terms and upstream media rights are checked.

The download page listed several same-date release variants for 15 November, all marked “Not built here yet”. Do not combine same-date variants, or assume the API response and a future bulk release have identical data. Every run must pin exactly one release ID.

## Observed release metadata

- API overview: https://watersearch.co.za/api
- OpenAPI 3.1 document: https://watersearch.co.za/api/v1/openapi.json
- Release endpoint: https://watersearch.co.za/api/v1/releases
- Bulk/download page: https://watersearch.co.za/data
- Selected release metadata: `2026-11-15-b7c711ba`
- Declared board coverage: 3 March 2025–15 November 2026
- No API response, bulk file, image or manifest bytes have been acquired in this PR. All SHA-256 values remain null/not applicable until exact bytes are captured and hashed.

## Critical semantics to preserve

1. **Supply-level is not tap-level.** A daily row reports what JW said about a named reservoir/tower/supply. It is not a pressure/flow/tap observation. A suburb may map to multiple supplies, and streets can differ.
2. **No report is not normal supply.** `no_report` means there was no JW board for that supply/day. Keep it in denominators as missing reporting, not adequate, no outage or zero loss.
3. **C41 is a segmentation rule.** Up to four consecutive no-board days between reports at `no_water` or `critical` do not break a derived outage span. Those days remain unobserved and are counted as `no_board_days`; do not turn them into observed no-water days.
4. **Censored durations are lower bounds.** If JW did not report the end, `end_at` is the last qualifying report and duration is minimum observed span. It is not a confirmed restoration time.
5. **Scheduled closure is not unplanned outage.** Keep recurring overnight closures separate from outage segments.
6. **A suburb summary is not a customer service measurement.** Preserve all supply rows linked to the suburb; do not reduce them to a household/tap pass/fail.

The exposed OpenAPI document states version 3.1 but uses empty object schemas for endpoint responses. Local strict schemas in this package are proposed contracts; they must be checked against exact pinned response bytes before use.

## Benchmark opportunity

Once rights-cleared rows are acquired, the history may help build a long-span **supply-reported stress trace** for continuity research. It can support reported no-water/critical days, explicit no-report coverage, censored duration distributions, and sensitivity analyses for the C41 gap threshold. It cannot establish household service, physical onset/restoration, flow/pressure, water quality, leak truth or repair effectiveness.

Compare it separately from:
- first-party operator notices in Mycelix #6110/#6111;
- instrumented SCADA/AMI and qualified water measurements;
- DWS/SAWS hydrometric and rainfall observations;
- independent community/tap-level or field evidence.

Do not double-count Watersearch plus the underlying JW board as independent corroboration.

## Artifacts

- `johannesburg-water-derived-datasets-v0.1.json`: metadata-only derived-source admission register.
- `johannesburg-water-derived-datasets-v0.1.schema.json`: strict registry schema.
- `watersearch-supply-daily-status-v0.1.schema.json`: separate raw/derived daily-row schema.
- `watersearch-derived-outage-span-v0.1.schema.json`: separate derived-span schema.
- `watersearch-derived-history-negative-controls-v0.1.json`: 12 synthetic lineage/gap/censoring controls.

All twelve negative controls are synthetic. This package does not acquire live or historical source bytes, author a data ingestion service, qualify an actual outage, or authorize service decisions.


## Reviewed API example (not pinned or acquired)

A public `/api/v1/supplies/hursthill-2-reservoir` response was inspected on 15 November 2026 under release `2026-11-15-b7c711ba`. The API reported:
- Hursthill 2 Reservoir; systems `central` and `commando`;
- board history 3 March 2025–15 November 2026, 334 reported days, 138 days classified as bad (`no_water` or `critical`), 573 board rows and 29 overnight-closure rows;
- for 2026, 106 bad days out of 228 reported days, against a reported city median of 6.5 bad days;
- latest board 15 November 2026 09:25 SAST: `warned`, on bypass, with wording that poor pressure/intermittent supply/no water *may* occur;
- last-ten-outage summary: median at least 17 hours, longest at least 64.833 hours, one censored interval.

These are **Watersearch-derived supply-reporting statistics**, not direct physical measurements. In particular, 106/228 uses reported days as denominator, not 365 calendar days; a `warned` state is not a `no_water`/ `critical` observation; the last-ten duration summary includes a censored lower-bound interval. The response was viewed in the public API page but was not downloaded, hashed or stored as a source fixture (`content_sha256: null`). Do not benchmark against this single example as an annual availability rate or household service result.
