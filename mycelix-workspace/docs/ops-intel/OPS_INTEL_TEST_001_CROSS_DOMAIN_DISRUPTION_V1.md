# OPS-INTEL-TEST-001 — CrossDomainDisruptionV1 seed corpus

**Status:** first public solver-visible seed candidate; synthetic fixture only  
**Version:** 0.1  
**Tracks:** [Mycelix #2942](https://github.com/Luminous-Dynamics/mycelix/issues/2942)  
**Coordinates:** [OPS-INTEL-001T / #2946](https://github.com/Luminous-Dynamics/mycelix/issues/2946), [independent verifier / #2950](https://github.com/Luminous-Dynamics/mycelix/issues/2950), and [Symthaea OPS-INTEL-005T / #5573](https://github.com/Luminous-Dynamics/symthaea/issues/5573)

## Scope

This PR freezes the first public **solver-visible** synthetic input seed, frontier deltas, structural conformance predicates, and mutation descriptors for the CrossDomainDisruptionV1 world. It is a fixture subject only; it does not add Rust, Holochain, network I/O, collection behavior, an operational recommendation engine, policy enforcement, or execution capability.

The seed covers SupplyChain, Energy, ServiceOperations, and OrganizationPolicy. F0 intentionally contains conflict, stale calibration, partial coverage, protected omission, and unknown source dependencies. F1 adds observations. F2 contains a simulation-only attempt and non-secret negative controls for absent/stale/mismatched mock permits. F3 contains a bounded synthetic outcome observation that conflicts with the interpretation of the attempt receipt.

## Files

- `CROSS_DOMAIN_DISRUPTION_V1_F0_SOLVER_VISIBLE.json` — the only input supplied to an F0 run.
- `CROSS_DOMAIN_DISRUPTION_V1_F1_DELTA.json` — supplied only for an F1 run or explicit replay using that frontier.
- `CROSS_DOMAIN_DISRUPTION_V1_F2_DELTA.json` — simulation-only attempt plus no-secret mock authority negative controls for absent, stale, and mismatched permits.
- `CROSS_DOMAIN_DISRUPTION_V1_F3_DELTA.json` — later outcome evidence; not available to F0/F1 computations.
- `CROSS_DOMAIN_DISRUPTION_V1_EXPECTED_PREDICATES.json` — 19 structural expectations, intentionally not a single preferred intervention.
- `CROSS_DOMAIN_DISRUPTION_V1_MUTATIONS.json` — 22 adversarial mutations and their expected failure/disposition class.

## Solver/evaluator separation

**No evaluator-only actual-world labels, hidden dependency graph, or private oracle values are committed in this public repository.** The public fixture must not masquerade as a hidden oracle simply because it contains later frontiers.

For noninterference tests, the independent harness must run the subject in a sandbox containing only the specific solver-visible frontier. It should execute paired evaluator worlds where only hidden evaluator data changes while the solver-visible bytes remain identical, then require byte-identical normalized deterministic output. The subject must not receive evaluator files, future deltas, their filenames, environment variables, or side channels.

A separate hidden evaluator package may be maintained outside this public repository by an independent evaluator. Its access path, identity, retention, and use must be bound by that evaluator's procedure. Do not put the payload in this repository and then call it hidden.

## Identity and hashing boundary

These JSON files freeze semantic fixture content, not canonical operational-projection encodings. Do **not** derive protocol identity from incidental JSON serialization, and do not claim these are canonical protocol golden hashes. Exact canonical framing, enum/tag encoding, collection ordering and known-answer byte/hash vectors belong to #2946. Once that contract is frozen, add independently verified golden vectors without silently changing this seed's semantic content.

The fixture files should be passed to solvers as distinct per-frontier inputs—not concatenated into one input document. The `frontier_ref` and parent fields express fixture lineage, not proof that an implementation enforced the frontier boundary.

## Core conformance rules

1. Preserve conflicting claims and their source/selector ancestry; newer does not automatically mean truer.
2. Preserve currentness, calibration, coverage, omission, and dependency uncertainty separately.
3. A change in a declared decision policy may change a candidate result but cannot rewrite factual projections.
4. Hard constraints are not soft penalties unless an explicitly versioned override policy states so.
5. Scenario and candidate outputs carry no authority and cannot enter native observed state.
6. A dispatch receipt is not proof of effect; outcome evidence does not automatically establish causation.
7. Historical F0/F1 inputs and outputs remain immutable after later evidence arrives.
8. The conformance suite must include a positive sensitivity control so a constant-output implementation cannot pass merely by being invariant.

## Evaluation ceiling

A future pass against these fixtures can establish only that an exact subject preserves the enumerated structural semantics for the tested cases. It does not prove external-world truth, source authenticity, source independence, optimal decisions, forecast accuracy, production security, scalable operation, legal admissibility, or superiority over any intelligence service or enterprise platform.

## Research anchors

The seed's analyst-facing evaluation direction is consistent with the UK Government's [all-source intelligence assessment framework](https://www.gov.uk/government/publications/intelligence-analysis-professional-development-framework/the-professional-development-framework-for-all-source-intelligence-assessment), which emphasizes source evaluation, audit trails, probabilistic judgments, hypotheses/scenarios, and bias mitigation, and with [NIST SP 800-150](https://csrc.nist.gov/pubs/sp/800/150/final) on scoped cyber-threat information-sharing goals, source/handling considerations, and distribution rules. These references inform the testing profile; they do not qualify this fixture or an implementation.

## Exact scope and non-claims

- No hidden evaluator data are included.
- No protocol golden hashes are claimed before #2946 freezes canonical encoding.
- No single candidate is declared universally correct.
- No production implementation or independent-verifier result is claimed.
- No live collection, action authority, or external effect is introduced.
