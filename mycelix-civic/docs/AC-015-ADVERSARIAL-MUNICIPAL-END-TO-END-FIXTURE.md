# AC-015 — Adversarial Municipal End-to-End Fixture

## Status

Draft qualification/evidence tranche stacked directly on AC-014. AC-015 adds no new governance authority and no new production inference primitive. It exists to attack the existing AC-005→AC-014 chain with realistic municipal procurement and beneficial-ownership data.

## Purpose

The anti-capture stack has accumulated enough architecture. AC-015 deliberately changes direction from abstraction to hostile evidence.

Its primary integration fixture traverses:

`OCDS JSON -> AC-005 ingestion -> AC-006 exact-identifier proposals -> explicit corroboration/review -> AC-008 qualification -> AC-014 full robustness matrix -> AC-010 envelope`.

BODS data participates in the same public-entity identity evidence while private-person identifiers remain suppressed.

## Municipal fixture

Two OCDS 1.1 release packages represent separate municipal publication snapshots:

- snapshot A at `recorded_at = 100`;
- snapshot B at `recorded_at = 200`.

Each has two awardees. One supplier appears under different source-local party records but shares public identifier:

`ZA-CIPC / SUP-ALPHA`.

A BODS 0.4 entity record independently declares the same public identifier and includes a private beneficial-owner record with passport-like test material.

AC-005 must project the legal entity and relationship while ensuring the private record ID and private identifier never appear in serialized public reconciliation output.

## Known arithmetic oracle

Before identity resolution there are four distinct awardee records with one award each:

`HHI_count_raw = (1^2 + 1^2 + 1^2 + 1^2) / 4^2 = 4/16`.

After the two OCDS supplier records for `SUP-ALPHA` are joined by qualified reversible identity evidence:

`HHI_count_resolved = (2^2 + 1^2 + 1^2) / 4^2 = 6/16`.

The success test asserts both exact ratios rather than only checking that the matrix builds.

## Robustness axes

The success fixture enables all four AC-014 axes:

- raw vs qualified identity;
- all valid vs corroborated-only evidence;
- baseline vs an early `[0, 150)` `recorded_at` window;
- count vs qualified award-value weighting.

This produces 16 scenarios (`2 * 2 * 2 * 2`).

The fixture intentionally leaves one of four award edges only `Declared`, while three receive a second independent provenance source and become `Corroborated`. This makes the evidence-admission axis nontrivial without causing empty strict scenarios.

## Monetary fixture

The four supplier-attributed award values are:

- 100 ZAR;
- 200 ZAR;
- 300 ZAR;
- 400 ZAR.

They are bound to the exact AC-003 award-edge IDs emitted by AC-005 and retain independent value provenance.

## Identity qualification

AC-006 first produces only `Proposed` exact-identifier links.

The fixture then explicitly models the later qualification steps rather than pretending proposal equals truth:

- authoritative registry lookup evidence;
- independent review reference and rationale;
- `Corroborated` status;
- time-bounded AC-008 qualification receipt;
- injected verifier checking the exact subject commitment and authority/policy references.

The verifier counts calls so the test proves AC-014 uses one frozen identity trust snapshot rather than re-verifying per scenario.

## Adversarial cases

AC-015 includes separate negative tests for:

### Duplicated joint-award value

Two supplier value records are mutated to share one upstream `award_ref`.

Expected result: AC-012 preflight rejects the set with `DuplicateAwardReference` before robustness analysis.

### Stale identity qualification

All identity receipts are expired before the AC-014 `evaluated_at` time.

Expected result: AC-008 returns `ReceiptOutsideValidityWindow`; AC-014 never reaches analytical interpretation.

### Unsupported OCDS extension

The valid OCDS fixture is mutated to declare an unsupported extension.

Expected result: AC-005 rejects it with `UnsupportedOcdsExtension`.

### Private BODS identifiers

The BODS fixture contains a passport-like record ID and private identifier.

Expected result: both are absent from serialized AC-005 output and public entity-identifier bindings, while a suppression warning remains visible.

## Non-goals

AC-015 does not claim that these fixtures establish real-world corruption detection accuracy, legal sufficiency, registry authenticity, calibrated false-positive rates, or municipal deployment readiness.

It is a deterministic adversarial integration corpus for the current architecture.

## Qualification gate

AC-015 is not qualified until:

1. the exact-subject integration tests execute and pass;
2. rustfmt passes;
3. warnings-denied Clippy passes including integration tests;
4. the raw and resolved exact HHI oracle values reproduce independently;
5. the 16-scenario Cartesian count reproduces;
6. verifier call count equals the number of supplied qualification receipts, not scenario count;
7. private BODS marker strings are absent from serialized projected output;
8. each adversarial mutation fails at the documented layer and error family;
9. no fixture mutates production source graph state in place outside the deliberate test evidence-upgrade clone;
10. review confirms no negative test is being interpreted as evidence of actual wrongdoing.

## Next step

After AC-015 passes executable qualification, the next improvement should be empirical rather than architectural: expand the adversarial corpus with randomized/property-based mutations, real public OCDS sample shapes, registry disagreement cases, supplier consortium/joint-award semantics, and false-positive/false-negative measurement. No new anti-capture authority should be added merely because the fixture suite becomes larger.
