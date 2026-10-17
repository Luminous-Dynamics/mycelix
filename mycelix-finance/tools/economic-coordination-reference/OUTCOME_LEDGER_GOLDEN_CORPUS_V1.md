# Procurement Outcome Ledger — golden corpus registry v1

**Status:** frozen fixture-ID registry and expected invariants; not all rows are implemented. The four end-to-end cases marked **authored** are in `tests/outcome_ledger_golden.rs`; the unit suite in `src/outcomes.rs` has additional authored adversarial coverage. None is declared passing until exact-head CI succeeds.

All fixtures use synthetic civilian procurement records. “LiveDomain” in a fixture means the test exercises the intended branch of the reference evaluator; it does not authenticate real evidence.

| Fixture ID | Scenario | Expected invariant | Status |
|---|---|---|---|
| OLG-001 | Full on-time accepted delivery | Final OTIF true; forecast error and cost variance match frozen values | Authored |
| OLG-002 | Same six events arrive in reverse order | Same canonical case report | Authored |
| OLG-003 | Exact retry of same event/claim | Idempotent; event count unchanged | Authored unit |
| OLG-004 | Same event ID, different payload | Identity conflict rejected | Authored unit |
| OLG-005 | Same claim ID, different payload and new event ID | Claim conflict rejected | Authored unit |
| OLG-006 | Synthetic acceptance event | Visible as excluded; zero trusted accepted quantity | Authored |
| OLG-007 | Correct a receipt with a superseding event | Old event remains; active projection uses replacement | Authored |
| OLG-008 | Corrected receipt preserves required quantity | Report remains internally consistent | Authored |
| OLG-009 | Correction changes event family | Rejected | Authored unit |
| OLG-010 | Correction targets an already superseded event | Second correction rejected | Authored unit target |
| OLG-011 | Correction crosses case or commitment | Rejected | Planned |
| OLG-012 | Replacement observed before predecessor | Rejected | Authored unit |
| OLG-013 | Duplicate active authorized commitments | Report returns explicit error | Planned |
| OLG-014 | Finalization with no matching commitment | Rejected during projection | Authored unit |
| OLG-015 | Duplicate active finalizations | Report returns explicit error | Planned |
| OLG-016 | Final currency differs from authorized currency | Finalization mismatch | Authored unit |
| OLG-017 | Finalization summary without detailed acceptance events | Summary supplies final quantities within commitment | Authored unit |
| OLG-018 | Detailed acceptance and finalization disagree | Finalization mismatch | Authored unit |
| OLG-019 | Accepted plus rejected exceeds receipt quantity | Quantity mismatch | Authored unit |
| OLG-020 | Accepted plus rejected exceeds commitment | Quantity mismatch | Authored unit |
| OLG-021 | Return quantity exceeds gross accepted quantity | Quantity mismatch | Authored unit |
| OLG-022 | Corrected receipt no longer reconciles accepted quantity | Projection refuses inconsistent report | Authored |
| OLG-023 | Event observed before occurrence | Invalid event | Authored unit |
| OLG-024 | Forecast point estimate outside declared interval | Invalid forecast event | Authored unit |
| OLG-025 | Forecast data cutoff after forecast time | Invalid forecast event | Planned |
| OLG-026 | Several pre-commitment forecasts | Latest eligible forecast selected deterministically | Planned |
| OLG-027 | Forecast issued only after commitment | Not selected as pre-commitment forecast | Planned |
| OLG-028 | Partial accepted quantity | OTIF false after valid finalization | Planned |
| OLG-029 | Accepted quantity delivered after due time | OTIF false | Planned |
| OLG-030 | No finalization event | OTIF and calibration metrics absent | Planned |
| OLG-031 | Open dispute after finalization | OTIF and forecast metrics suppressed | Authored unit |
| OLG-032 | Independent verifier resolves same-case dispute | Active dispute removed only with a trusted resolution | Authored unit |
| OLG-033 | Dispute resolution references another case | Rejected | Authored unit |
| OLG-034 | Non-independent issuer submits dispute resolution | Rejected by role contract | Authored unit |
| OLG-035 | Duplicate resolution of same active dispute | Rejected | Authored unit target |
| OLG-036 | Contested finalization | Not counted as trusted final outcome | Planned |
| OLG-037 | Stale or unknown-origin event | Excluded from trusted totals and surfaced in counts | Planned |
| OLG-038 | Supplier and buyer submit same real-world fact under different source IDs | Ledger cannot infer equivalence; disclose limitation, do not claim global deduplication | Negative control |
| OLG-039 | Accepted/rejected arithmetic overflow within one event | Invalid event, no wraparound | Authored unit validation path |
| OLG-040 | Frozen case report serialized/replayed on different input order | Byte-identical canonical output once serializer is qualified | Planned |

## Qualification rules

- Preserve every fixture ID and negative control after implementation.
- Each implemented fixture records the tested source revision and input digest.
- A test using the production report twice is not an independent oracle. Golden outcomes should contain literal expected fields or use a separate test-local replay implementation.
- Random generation starts only after the independent small-instance oracle is qualified. Pin generator version, seed, case count and corpus digest.
- The fixture registry and the executable coverage map must stay separate: a row is not “covered” merely because it appears in this document.
- CI status belongs to an exact commit SHA. Queued/pending is not PASS; a failure before the Rust test command is not a Rust test failure.
