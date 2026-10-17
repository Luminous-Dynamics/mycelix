# Revision-Fenced Capacity Reservation — golden corpus registry v1

**Status:** 43 stable fixture IDs; not all rows are implemented. No fixture is marked Passing without exact-head evidence.  
**Tracking issue:** [PROC-RESERVE-CAS-001 #5153](https://github.com/Luminous-Dynamics/mycelix/issues/5153).  
**Implementation:** [reservation_cas.rs](https://github.com/Luminous-Dynamics/mycelix/blob/feat/procurement-reservation-cas-v1/mycelix-finance/tools/economic-coordination-reference/src/reservation_cas.rs).

Eighteen module test functions plus one integration oracle test are authored. The module tests exercise ReservationBook directly; tests/reservation_cas_oracle.rs separately computes expected reserve acceptance and pool accounting for 108 short sequences without calling ReservationBook helpers. The oracle is authored, not yet qualified as passing. Fixtures marked Authored indicate test presence, not a pass.

| Fixture ID | Scenario | Expected invariant | Status |
|---|---|---|---|
| RSC-001 | 100 attempts for one unit all present revision 1/fence 1 | At most one reserve succeeds; occupied quantity ≤ 1 | Authored |
| RSC-002 | Exact retry of same reserve command/key | Original receipt returned; no second allocation/event | Authored |
| RSC-003 | Same idempotency key with changed quantity | Conflict rejected without mutation | Authored |
| RSC-004 | Stale pool revision | Reserve fails without journal growth | Authored |
| RSC-005 | Current revision but stale fence | Reserve fails without journal growth | Authored |
| RSC-006 | Wrong capacity snapshot digest | Reserve fails closed | Authored |
| RSC-007 | Capacity evidence expired at attempted reserve time | No reservation receipt/event | Authored |
| RSC-008 | Verified synthetic capacity origin | Never promoted to live capacity | Authored |
| RSC-009 | Expire command before lease expiry | Rejected; active reservation unchanged | Authored |
| RSC-010 | Expire command at exact lease boundary | Explicit Expired event frees capacity | Authored |
| RSC-011 | Committed reservation release attempt | Rejected; capacity remains occupied | Authored |
| RSC-012 | Consume before commit | Rejected; no mutation | Authored |
| RSC-013 | Commit then consume | Reservation becomes Consumed; capacity remains occupied | Authored |
| RSC-014 | Exact commit-command retry | Original event returned; no duplicate event | Authored |
| RSC-015 | Commit at expiry boundary | Rejected; explicit expiry still required | Authored |
| RSC-016 | Release then reserve freed capacity | New reserve requires current revision/fence | Authored |
| RSC-017 | Duplicate pool identity | Second registration cannot overwrite first | Authored |
| RSC-018 | Duplicate reservation ID under new idempotency key | Rejected | Authored |
| RSC-019 | Reuse transition command ID for different action | Rejected | Authored |
| RSC-020 | Pool evidence expires after registration but before reserve | Rejected without event | Authored |
| RSC-021 | Transition timestamp precedes reservation creation | Rejected; no mutation | Authored |
| RSC-022 | Duplicate reserve command ID under a different key | Rejected | Planned |
| RSC-023 | Zero reservation quantity | Rejected | Planned |
| RSC-024 | Empty policy or manifest digest | Rejected | Planned |
| RSC-025 | Lease expiry equal to issue time | Rejected | Planned |
| RSC-026 | Capacity quantity reached exactly | Accepted | Planned |
| RSC-027 | Capacity exceeded by one after current-revision refresh | Rejected | Planned |
| RSC-028 | Two distinct units use separate pool IDs | Accounting remains isolated | Planned |
| RSC-029 | Same pool ID registered again with a changed capacity ceiling | First snapshot not overwritten | Authored |
| RSC-030 | Evidence observed after reservation command time | Rejected | Planned |
| RSC-031 | Revision at u64 maximum | Revision overflow fails closed, no mutation | Planned |
| RSC-032 | Fencing token at u64 maximum | Fence overflow fails closed, no mutation | Planned |
| RSC-033 | Event sequence arithmetic boundary | No sequence wraparound | Planned |
| RSC-034 | Two successful mutations use increasing revisions/fences | Strictly monotonic revision and fencing token | Planned |
| RSC-035 | Release attempted exactly at expiry | Rejected; explicit expire transition required | Planned |
| RSC-036 | Consumed reservation release attempt | Rejected; consumed capacity preserved | Planned |
| RSC-037 | Explicit release exact retry under same command ID | Idempotent event replay | Planned |
| RSC-038 | Two concurrent operations against same revision | One mutation succeeds; stale sibling fails | Authored |
| RSC-039 | Failed commit followed by valid expiry command | Failed commit creates no journal event; expiry is recorded | Planned |
| RSC-040 | Restart/replay from persisted event stream | Reconstructed states and pool totals match oracle | Planned |

| RSC-041 | Requested lease expiry is later than the capacity snapshot validity | Lease rejected without reservation event | Authored |
| RSC-042 | Two pool mutations presented with decreasing timestamps | Second mutation rejected with no journal growth | Authored |
| RSC-043 | Exhaustive three-command quantity combinations for capacities 1–4 | 108 sequence outcomes and pool totals agree with separate test-local oracle | Authored |

## Qualification rules

- Keep all 43 fixture IDs stable and preserve all negative controls.
- Mark Authored only if an executable test covers the stated invariant.
- Mark Passing only with the exact source revision and exact-head CI result.
- Qualify the 108-sequence test-local oracle on exact head before larger generated testing.
- After oracle qualification, run at least 10,000 deterministic schedules; pin generator version, seed, schedule manifest and digest.
- The 100-contender unit test is a deterministic model check against one initial revision, not a real concurrent-store stress benchmark.
