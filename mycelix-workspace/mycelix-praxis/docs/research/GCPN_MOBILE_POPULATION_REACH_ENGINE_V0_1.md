# GCPN Mobile-Population Reach Engine v0.1 — Stage B

**Status:** Rust source authored; compile/test/CI remain outstanding.  
**Parent issue:** #5654.  
**Specification PR:** #5655.  
**Package:** `benchmarks/gcpn-mobile-population-reach`.  
**Dependencies:** none; isolated Cargo package.

## 1. Implemented aggregate-only core

This increment converts the mobility specification into an aggregate Rust implementation. It does not represent any individual.

Implemented structures:
- cohort class: resident, commuter, transit, seasonal, synthetic displaced, movement-constrained;
- population cells keyed only by synthetic cohort and coarse zone;
- aggregate flows between coarse zones;
- aggregate reach cells keyed by cohort, zone and time bucket;
- explicit unique timely receipt union (optional/unknown);
- explicit unique accessible receipt union;
- channel receipt totals kept separate from unique receipt union;
- synthetic cross-jurisdiction authority-handoff status;
- minimum-cell suppression and a simple single-cell differencing red-team control.

There are no person/device IDs, GPS traces, persistent movement histories or identity-linked records.

## 2. Aggregate movement conservation

`apply_aggregate_flows` verifies each origin has sufficient aggregate population, destination zones exist, and aggregate counts are conserved exactly across the transition.

The small CLI fixture moves 65 members of a synthetic commuter cohort from a home zone to a work zone while retaining a separate movement-constrained cohort in place. This is an illustrative synthetic test, not an estimate of real movement.

## 3. Reach accounting: a critical deduplication distinction

A channel receipt total is not the same as a unique cohort union.

If one aggregate cohort receives two channel receipts per reached person, channel totals can exceed unique reach. The evaluator therefore accepts:
- `channel_receipt_count`: raw aggregate channel receipt count;
- `unique_timely_receipts`: an aggregate union supplied by an independently governed deduplication process;
- `unique_accessible_receipts`: the aggregate union receiving an appropriate accessible representation.

The evaluator never substitutes channel receipt sums for unique reach. If the unique union is missing, effective reach and duplicate-above-union are reported as unknown (not zero). It rejects accessible unique counts that exceed timely unique counts and channel receipt totals smaller than the unique union.

Accessible-format shortfall among cohorts where accessibility is required is reported separately. Cohort-level and worst-cohort timely reach remain visible.

Metrics are exposure/person-time weighted across zone/time cells. They are **not** claims of unique persons across a day, receipt by a particular person or comprehension.

## 4. Cross-jurisdiction handoff

The synthetic handoff validator distinguishes:
- invalid or time-reversed handoff records;
- scope mismatch;
- awaiting acknowledgement;
- acknowledged relay;
- acknowledged reissue permission.

A relay acknowledgement without reissue permission does not grant the receiving party authority to issue a new message. This is a synthetic conformance rule and not a substitute for jurisdiction-specific law or agreements.

## 5. Privacy negative controls

The package implements minimum-cell suppression and a deliberately narrow red-team check for two published aggregate query sets whose difference isolates one suppressed small cell.

This is only a simple differencing negative control. Minimum-cell suppression is not differential privacy and does not protect against arbitrary query composition, auxiliary information or all re-identification attacks. Production privacy requires a threat model, governed query policy, privacy review and potentially formal privacy mechanisms.

## 6. Authored tests

Twelve Rust tests are included:
- aggregate conservation while movement-constrained cohorts remain represented;
- overdrawn and unknown flows fail closed;
- duplicate population cells rejected;
- self-zone movement and aggregate count overflow are rejected;
- channel receipts do not substitute for unique reach;
- unknown union keeps effective and duplicate reach unknown;
- accessible reach cannot exceed unique timely reach;
- accessible union cannot be claimed when timely union is unknown;
- cross-jurisdiction reissue requires acknowledgement, valid scope and permission;
- time-reversed handoff rejected;
- minimum-cell suppression applied;
- simple aggregate differencing attack detected;
- duplicate reach cells rejected and duplicate receipt excess reported.

The source currently contains the test cases as authored; test execution is not claimed.

## 7. Limitations and next steps

Not implemented:
- full aggregate cohort-transition DGP matrix and uncertainty intervals;
- privacy budget/query ledger or differential privacy;
- multidimensional differencing/composition attacks;
- actual CAP parsing or transport;
- statistical estimation of flow uncertainty;
- accessibility/language co-design;
- locally measured costs;
- human participant observations.

No actual population movement data, telecom subscriber data or location feeds are used.

## 8. Validation status

- Isolated Cargo manifest, Rust library and CLI are on the feature branch.
- Source-level checks and profile/document reviews performed.
- No Cargo build, Rust test execution, CLI run or CI pass claimed because this runtime has no Cargo or rustc.
- No real population reach or privacy guarantee claimed.

## 9. Sources and boundary

The research direction is based on UNDRR/WMO/IFRC/ITU's *Early Warning Systems for Mobile Populations: Operational Guidance* (15 July 2026): https://www.undrr.org/publication/documents-and-publications/early-warning-systems-mobile-populations-operational

This remains a synthetic aggregate benchmark. No personal tracking, migration-status inference, recipient targeting, live alerting, operational cross-border issuance or responder direction is introduced.
