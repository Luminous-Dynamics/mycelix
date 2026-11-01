# GCPN Integrated Synthetic Evaluation v0.2

**Status:** integrated synthetic evaluation package; non-operational.  
**Parent:** issue #5618.  
**Package:** `benchmarks/gcpn-integrated-evaluation`.  
**Dependencies:** four local Rust packages in this repository; no third-party Cargo dependencies.

## 1. Why integrate now

The component benchmarks each test a different failure class:
1. civil warning-record integrity and authority boundaries;
2. topology reachability under service and link failures;
3. last-mile unique delivery and accessible-format coverage;
4. warning-to-action evidence and outcome-claim discipline.

The integrated package composes their public APIs and verifies cross-package invariants. It does not merge their metrics into a single score.

## 2. Four independent gates

| Component | Frozen cases | Required result |
|---|---:|---|
| Civil record conformance v0.1.1 | 9 | All hard integrity gates pass |
| Topology resilience v0.1 | 11 scenarios × 3 architectures | Same denominator across architectures; authority gate blocks all; declared local-first profiles preserve local delivery |
| Last-mile multi-channel v0.1 | 5 receiver cohorts × 10 channel scenarios | Unique and accessible reach stay within denominators; duplicate receipts do not inflate unique people |
| Warning-to-action v0.1.1 | 11 cases | Negative-control oracle matches exactly; funded, unfunded and abstention positive controls pass independently |

The warning-to-action fixture intentionally contains failures. A correct oracle must detect those failures; its fixture is not expected to pass every hard gate. The integration report distinguishes fixture-oracle success from the valid positive-control status.

## 3. Integration invariants

### Record integrity

The civil-record package must continue to reject unauthorized alerts, unbound authority assertions, hidden uncertainty, provenance leaks, false source-family independence and invalid state changes.

### Topology

Each abstract topology sees the same receiver denominator for each scenario. A competent-authority failure blocks every architecture. Local-first delivery is credited only where the local path, local authority and local last-mile channel are explicitly available.

### Last mile

For every scenario:
- accessible receivers cannot exceed unique receivers;
- unique receivers cannot exceed intended receivers;
- uncovered receivers equal intended minus unique reached;
- channel receipts cannot be lower than unique reach;
- duplicate channel receipts equal channel receipts minus unique reach.

The 2,700-person cohort fixture is synthetic and independent from the 2,000-person topology fixture. They are separate controlled experiments, not one shared population.

### Warning-to-action

The integrated runner verifies the known negative-control oracle and the separate positive controls:
- valid funded completed chain;
- valid authorized unfunded action where the protocol does not require a separate finance-release record;
- valid authorized abstention.

The benchmark records evidence only. It cannot issue an alert, activate a protocol, move funds or direct action.

## 4. Reproduction

From repository root:

```bash
cargo test --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-integrated-evaluation/Cargo.toml
cargo run --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-integrated-evaluation/Cargo.toml
```

The CLI reports each component separately and prints an integrated fixture-reproduction status. This status is an AND of declared fixture invariants, not a weighted quality score.

## 5. Fixture provenance

The component fixture byte fingerprints are pinned within their own packages or explicitly audited here. FNV-1a values are regression identifiers, not cryptographic signatures or provenance authentication.

| Fixture | Rows (excluding header) | FNV-1a 64-bit |
|---|---:|---|
| Civil record `gcpn-001-v0.1.1.csv` | 9 | `0x8e5118bf74fc7b13` |
| Topology `topology-fault-matrix-v0.1.csv` | 11 | `0xe6f6679f94c93e31` |
| Last-mile `receiver-cohorts-v0.1.csv` | 5 | `0x59f956045ca16c3d` |
| Last-mile `channel-fault-matrix-v0.1.csv` | 10 | `0x2a7269d9a7d4caee` |
| Warning-to-action `action-chain-v0.1.1.csv` | 11 | `0x4084062a6c04598f` |

All fixture values are synthetic. Fingerprints guard against accidental edits; they do not authenticate authorship or establish real-world accuracy.

## 6. Why there is no single GCPN score

A weighted composite would permit invalid compensation:
- high delivery counts could mask an authority failure;
- high average accessibility could mask an excluded cohort;
- excellent local continuity could mask failure of the competent authority gate;
- many completed actions could mask a suppressed grievance or unsupported impact claim.

GCPN therefore exposes each module's hard gates and numerical metrics separately. The integrated reproduction predicate only checks whether the frozen test oracles remain consistent; it does not rank a real system.

## 7. Industry / SOTA comparison protocol

This package is not itself a SOTA evaluation. It creates a repeatable test harness for subsequent compatible comparisons.

Future comparisons must be made layer by layer:
- CAP 1.2 semantic conformance and optional provenance sidecars;
- national/regional warning-system interoperability and local continuity;
- channel portfolios and receiver-level accessibility;
- warning-to-action protocol timing, reach, grievances, cost and outcome evidence.

Use equivalent alert/event definitions, receiver denominators, hazard categories, forecast horizons, cost boundaries and test conditions. Keep held-out events separate from any calibration or threshold-selection data. Report each hazard/horizon/region/cohort separately and disclose missing/abstained records. Do not rank named national or regional programmes without formal permission, compatible adapters and independent evaluation.

## 8. Relationship to AMHEWAS and SADC SHOC

AMHEWAS and SADC SHOC already have roles in continental and regional coordination. GCPN is designed as optional conformance and evidence tooling that could complement national systems and regional mechanisms, not replace them. No partnership, data access, endorsement or live interoperability is established by this repository work.

The SADC MHEWS consultancy notice issued 28 October 2026 seeks improvements to national capacity and interoperability between national warning systems, SHOC and AMHEWAS, including CAP and responsible AI/ML. Its closing date is 17 November 2026 at 12:00 PM. This is an ecosystem demand signal, not a claim that GCPN is selected or eligible.

References:
- SADC MHEWS consultancy: https://sadc.int/procurement-opportunities/consultancy-undertake-capacity-building-arising-identified-mhews-gaps-and
- WMO, 35 EW4All roadmaps (29 Oct 2026): https://wmo.int/media/update/from-global-ambition-national-action-take-aways-from-35-ew4all-roadmaps
- ITU Common Alerting Protocol and call to action: https://www.itu.int/en/ITU-D/Emergency-Telecommunications/Pages/Common-Alerting-Protocol-and-Call-to-Action.aspx
- UNDRR AMHEWAS programme: https://www.undrr.org/about-undrr/where-we-work/regional-office-africa/amhewas

## 9. NRVDEF remains separate

The nuclear risk-reduction and verification-evidence programme is intentionally not merged into the civilian-warning evaluation. It has a different institutional, treaty and verification boundary. Its executable custody state-machine benchmark remains in PR #5614 and must not be treated as another GCPN score or as a means of deciding physical truth or treaty compliance.

## 10. Validation status and non-claims

The source and fixture structure has been reviewed through repository retrieval; the component fixtures were independently checked for row counts, field widths, unique IDs and expected arithmetic. The integration Rust source has been structurally reviewed.

The current authoring environment does not provide `cargo` or `rustc`. Compilation and test execution are not claimed. Independent Rust execution and CI remain necessary.

This package is synthetic-only. It does not operate a warning system, connect to national services, forecast hazards, set thresholds, issue public alerts, activate actions, release funding, allocate assistance, or certify impact.
