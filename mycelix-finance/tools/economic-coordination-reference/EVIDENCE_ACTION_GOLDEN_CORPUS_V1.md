# Decision-Sensitive Procurement Evidence Actions — golden corpus v1

**Status:** 40 stable fixture IDs. Authored means a test is present; Planned means coverage remains to be implemented. No case is marked Passing without exact-head evidence.  
**Tracking:** [PROC-INFO-001 #5163](https://github.com/Luminous-Dynamics/mycelix/issues/5163).  
**Implementation:** [evidence_actions.rs](https://github.com/Luminous-Dynamics/mycelix/blob/feat/procurement-evidence-actions-v1/mycelix-finance/tools/economic-coordination-reference/src/evidence_actions.rs).

Eight module test functions are authored. They cover frontier sensitivity vs a no-change query, the decision deadline, evidence staleness, duplicate claim alternatives, mixed acquisition currency, inconsistent request axes, duplicated outcomes and canonical subject ordering. The module has not been qualified by exact-head execution.

| Fixture ID | Scenario | Expected invariant | Status |
|---|---|---|---|
| EAC-001 | Outcome branches change the baseline frontier | Meaningful frontier-sensitivity action is recommended | Authored |
| EAC-002 | Outcome branches all preserve baseline frontier | No-decision-value action is not recommended | Authored |
| EAC-003 | Acquisition completes after decision cutoff | Action excluded with deadline disposition | Authored |
| EAC-004 | Evidence expires before latest completion | Action excluded as stale by completion | Authored |
| EAC-005 | Two acquisition methods cover same claim/version | At most one method recommended | Authored |
| EAC-006 | Action cost currencies differ | Mixed acquisition currency rejected | Authored |
| EAC-007 | Branch frontier changes request axes | Fail closed on inconsistent request/unit axes | Authored |
| EAC-008 | Different outcome IDs return same canonical frontier | Duplicate branches do not inflate distinct-frontier count | Authored |
| EAC-009 | Reordered subject IDs in action | Identical canonical report | Authored |
| EAC-010 | Permuted outcomes have the same frontiers | Identical sensitivity counts and ordering | Planned |
| EAC-011 | Two frontier vectors reorder shortage entries | Axis vectors canonicalized by request ID/unit | Authored |
| EAC-012 | Empty action catalogue | Valid empty recommendation set | Planned |
| EAC-013 | Empty baseline frontier | Rejected as invalid baseline | Planned |
| EAC-014 | Empty outcome set | Action rejected | Planned |
| EAC-015 | Duplicate outcome ID | Action rejected | Planned |
| EAC-016 | Duplicate action ID | Problem rejected | Planned |
| EAC-017 | Empty source authority class | Action rejected | Planned |
| EAC-018 | Empty covered-subject set | Action rejected | Planned |
| EAC-019 | Duplicate covered subject IDs | Subjects canonicalized and deduplicated | Authored |
| EAC-020 | Outcome costs use mixed procurement currencies | Rejected before ranking | Planned |
| EAC-021 | Branch has no feasible allocation frontier | Empty frontier handled as a distinct declared outcome | Planned |
| EAC-022 | Baseline/branch has duplicate objective vectors | Canonical objective set avoids duplicate-count inflation | Authored |
| EAC-023 | A branch contains duplicate request axis | Rejected | Planned |
| EAC-024 | Same request ID uses inconsistent units in branches | Rejected | Authored |
| EAC-025 | Evidence validity ends before completion | Action marked stale by completion | Authored |
| EAC-026 | Completion time precedes evaluation time | Rejected as invalid timeline | Planned |
| EAC-027 | Decision deadline precedes evaluation time | Problem rejected | Planned |
| EAC-028 | Equal impact, cost and latency across distinct claims | Stable action-ID tie-break | Planned |
| EAC-029 | Lower-cost action has strictly lower impact | Neither falsely called dominant unless all criteria agree | Planned |
| EAC-030 | One action is no worse in impact/cost/latency and strictly better | Dominated action is labelled and not recommended | Planned |
| EAC-031 | Raw outcome count inflated by duplicate outcomes | Unique frontier count used | Authored |
| EAC-032 | Outcome count is high but all outcomes preserve baseline | No-decision-value action excluded | Authored |
| EAC-033 | Different outcomes shift shortage from urgent to routine request | Request axes remain separate | Planned |
| EAC-034 | Same claim/version has a cheap slow and expensive fast method | One deterministic method only; alternatives visible | Authored |
| EAC-035 | Shared observation covers multiple offer/mapping IDs | Coverage is preserved and canonicalized | Authored |
| EAC-036 | Acquisition cost arithmetic or score comparison near integer boundary | No unchecked scalar aggregation or overflow | Planned |
| EAC-037 | No declared probability model | No expected-value or probability-weighted result emitted | Authored |
| EAC-038 | Inconclusive observation has unchanged frontier | Not treated as proof of ineligibility | Planned |
| EAC-039 | Late observation would change the frontier substantially | Still excluded when completion exceeds deadline | Authored |
| EAC-040 | Frontier symmetric difference is nonzero but decision objectives are incomparable | Expose branch-level frontiers; do not call the metric calibrated utility | Planned |

## Qualification sequence

1. Exact-head formatting, compile and tests.
2. Add a separately implemented exhaustive oracle over literal action/outcome tables.
3. Qualify the oracle on all small candidate/outcome tables before generating larger cases.
4. Preserve unknown/inconclusive outcomes as distinct negative controls.
5. After oracle qualification, execute at least 10,000 deterministic generated micro-instances with pinned seed, generator, catalogue digest and source revision.
6. Record recommendations and per-branch frontiers; do not emit calibrated EVSI/expected-utility claims without a separately qualified probability model.
