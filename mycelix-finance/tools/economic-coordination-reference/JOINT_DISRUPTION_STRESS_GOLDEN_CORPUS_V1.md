# Joint Disruption Stress v1 — golden corpus

**Status:** 48 stable fixture IDs registered. “Authored” means a named test exists in the module or independent integration oracle; it does not mean the test passed CI. Planned fixtures are not implemented unless separately marked.  
**Tracking:** [PROC-RESILIENCE-002 #5169](https://github.com/Luminous-Dynamics/mycelix/issues/5169).  
**Benchmark contract:** [JOINT_DISRUPTION_STRESS_BENCHMARKS_V1.md](https://github.com/Luminous-Dynamics/mycelix/blob/feat/procurement-joint-disruption-stress-v1/mycelix-finance/tools/economic-coordination-reference/JOINT_DISRUPTION_STRESS_BENCHMARKS_V1.md).

| Fixture IDs | Scenario / invariant | Coverage status |
|---|---|---|
| JDS-001 | Supplier outage removes all offers from the failed supplier | Authored module test |
| JDS-002 | Shared-pool outage removes every offer drawing from that pool | Authored module test |
| JDS-003 | Unknown supplier failure ID is rejected | Authored module test |
| JDS-004 | Unknown capacity-pool failure ID is rejected | Authored module test |
| JDS-005 | Duplicate scenario ID is rejected | Authored module test |
| JDS-006 | Duplicate supplier failure ID is rejected | Authored module test |
| JDS-007 | Scenario results use canonical scenario-ID order | Authored module test |
| JDS-008 | Failure-domain list permutations preserve the report | Authored module test |
| JDS-009 | Aggregate budget exhaustion returns no report | Authored module test |
| JDS-010 | Empty scenario catalogue is rejected | Authored module test |
| JDS-011 | Independent oracle: no-failure two-request/two-offer control | Authored integration oracle |
| JDS-012 | Independent oracle: one supplier outage | Authored integration oracle |
| JDS-013 | Independent oracle: shared-pool outage | Authored integration oracle |
| JDS-014 | Independent oracle: two correlated supplier failures | Planned |
| JDS-015 | Same supplier represented by several distinct offers | Planned |
| JDS-016 | Several suppliers share one physical capacity pool | Planned |
| JDS-017 | Different capacity pools remain independent after one outage | Planned |
| JDS-018 | One request has no surviving compatible offers | Planned |
| JDS-019 | No offers survive any declared scenario | Planned |
| JDS-020 | Failure catalogue exceeds maximum scenario count | Planned |
| JDS-021 | Blank scenario-set identifier | Planned |
| JDS-022 | Blank scenario-set digest | Planned |
| JDS-023 | Blank scenario ID | Planned |
| JDS-024 | Blank per-scenario manifest digest | Planned |
| JDS-025 | Duplicate scenario manifest digest | Planned |
| JDS-026 | Duplicate failed pool ID is rejected | Planned |
| JDS-027 | Blank failed supplier ID is rejected | Planned |
| JDS-028 | Blank failed pool ID is rejected | Planned |
| JDS-029 | Mixed currency in frozen base problem remains rejected even if the bad offer would be filtered | Planned |
| JDS-030 | Invalid offer cannot be hidden by failure filtering | Planned |
| JDS-031 | Invalid capability mapping cannot be hidden by failure filtering | Planned |
| JDS-032 | Mapping to a failed offer is removed before optimization | Planned |
| JDS-033 | Unfailed offers retain their mappings | Planned |
| JDS-034 | Failed supplier and failed pool overlap without duplicate exclusions | Planned |
| JDS-035 | Multiple requests contend for one pool in the same scenario | Planned |
| JDS-036 | Per-request shortage dimensions survive scenario transformation | Planned |
| JDS-037 | Common-currency cost axis survives scenario transformation | Planned |
| JDS-038 | Base problem with no requests is rejected | Planned |
| JDS-039 | Zero scenario search budget is rejected | Planned |
| JDS-040 | Scenario-specific optimizer budget exhaustion is an error | Planned |
| JDS-041 | Aggregate budget exhausted after one completed scenario returns no partial report | Planned |
| JDS-042 | Aggregate node-count addition overflow fails closed | Planned |
| JDS-043 | Scenario ordering permutation preserves complete result | Planned |
| JDS-044 | Offer ordering permutation preserves canonical results | Planned |
| JDS-045 | Mapping ordering permutation preserves canonical results | Planned |
| JDS-046 | Failure-domain ordering permutation preserves canonical results | Planned |
| JDS-047 | 32-scenario maximum-size catalogue remains within explicit global cap or fails closed | Planned |
| JDS-048 | Repeated scenario with different ID is not interpreted as a probability weight | Planned |

A planned fixture may not be called passing and does not satisfy the independent-oracle requirement until implemented and verified. Benchmark reports should identify the exact subset actually executed.
