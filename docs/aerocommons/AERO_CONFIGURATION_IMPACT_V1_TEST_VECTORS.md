# AeroCommons Configuration Impact v1 Test Vectors

Each vector describes the expected classification for a single change/evidence relationship. These are protocol tests, not certification tests.

## 1. Direct subject change
- Evidence: E1
- Subject: A1
- Change: A1.geometry
- Validity envelope: none
- Expected: RequiresReview or Invalidated according to declared evidence semantics
- Diagnostic: changed subject dependency

## 2. Covered parameter change
- Evidence: E2
- Subject: A1
- Dependency: A1.thickness
- Validity envelope: 2.0 mm through 2.5 mm
- Change: 2.1 mm -> 2.2 mm
- Expected: ConditionallyValid

## 3. Out-of-envelope parameter change
- Evidence: E3
- Dependency: A1.thickness
- Validity envelope: 2.0 mm through 2.5 mm
- Change: 2.1 mm -> 2.8 mm
- Expected: Invalidated or RequiresReview, never Unaffected

## 4. Undeclared dependency
- Evidence: E4
- Change: A hidden dependency is modified
- Expected: Unknown
- Required diagnostic: missing dependency

## 5. Upstream invalidation
- E5 depends on E4
- E4 becomes Invalidated
- E5 has no independent basis
- Expected: RequiresReview or Invalidated

## 6. Unrelated metadata change
- Evidence: E6
- Change: human-readable label only
- No semantic dependency changes
- Expected: Unaffected

## 7. Failed physical test
- Evidence: E7
- Result: failed
- Later configuration change occurs
- Expected: failure remains preserved and queryable
- Forbidden behavior: deletion or conversion to success through attestation

## 8. Reproduction is not automatic independence
- E8 and E9 use the same operator, toolchain, dataset, and physical artifact
- Expected: distinct evidence records but no automatic independence claim

## 9. Determinism
- Same configuration, ChangeSet, evidence graph, and ruleset supplied twice
- Expected: byte-equivalent canonical result

## 10. Authority boundary
- An agent consensus record claims that an invalidated test should remain valid
- Expected: consensus does not override the impact classification or physical evidence state