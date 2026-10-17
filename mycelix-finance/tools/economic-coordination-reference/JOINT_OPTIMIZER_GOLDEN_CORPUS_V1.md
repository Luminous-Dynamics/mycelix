# Exact Joint Allocation — golden corpus registry v1

**Status:** 40 stable fixture IDs. Authored means test presence, not a pass; rows not currently covered remain Planned. No row is marked Passing without exact-head evidence.  
**Tracking issue:** [PROC-JOINT-EXACT-001 #5151](https://github.com/Luminous-Dynamics/mycelix/issues/5151).  
**Implementation:** [joint_optimizer.rs](https://github.com/Luminous-Dynamics/mycelix/blob/feat/procurement-joint-exact-v1/mycelix-finance/tools/economic-coordination-reference/src/joint_optimizer.rs).

Six module tests are authored, including the binary-mask oracle, explicit search-bound error, mixed-currency rejection, input-order invariance, aggregate order-lot handling, and omission of non-mapped edges. They are not yet qualified as passing.

| Fixture ID | Scenario | Expected invariant | Status |
|---|---|---|---|
| JEX-001 | Two requests, two offers, one shared pool, capacity 1 | Pareto frontier agrees with independent 16-mask oracle | Authored |
| JEX-002 | Same binary problem, shared capacity 2 | Pareto frontier agrees with independent oracle | Authored |
| JEX-003 | Search node budget deliberately too small | Explicit incomplete-search error; no partial frontier | Authored |
| JEX-004 | Mixed USD/EUR offers in cohort | Reject before comparing total costs | Authored |
| JEX-005 | Reversed request/offer/pool/mapping input order | Identical canonical report and node count | Authored |
| JEX-006 | Aggregate order lot split over two requests | Only aggregate-valid offer quantities appear on frontier | Authored |
| JEX-007 | Request/offer pair without explicit mapping | No positive allocation is emitted for that pair | Authored |
| JEX-008 | Zero eligible offers | Zero-allocation frontier reports all shortages | Planned |
| JEX-009 | One request fulfilled by two offers | Total demand reconciles; per-offer ceilings hold | Planned |
| JEX-010 | Same physical pool referenced by several offers | Capacity counted once | Planned |
| JEX-011 | Request budget binds optimum | No candidate breaches per-request cost ceiling | Planned |
| JEX-012 | Offer unit cost at zero | Budget bound does not divide by zero | Planned |
| JEX-013 | Arithmetic boundary in candidate costs | Fail closed; no wrapped objective | Planned |
| JEX-014 | Duplicate request ID | Invalid problem rejected before search | Planned |
| JEX-015 | Duplicate offer ID | Invalid problem rejected before search | Planned |
| JEX-016 | Duplicate pool ID | Invalid problem rejected before search | Planned |
| JEX-017 | Missing pool referenced by offer | Invalid problem rejected before search | Planned |
| JEX-018 | Stale mapping | No positive allocation uses the mapping | Planned |
| JEX-019 | Synthetic mapping presented as live | No positive allocation uses the mapping | Planned |
| JEX-020 | Mapping observed in future | No positive allocation uses the mapping | Planned |
| JEX-021 | Expired offer evidence | Invalid frozen manifest rejected | Planned |
| JEX-022 | Expired capacity evidence | Invalid frozen manifest rejected | Planned |
| JEX-023 | Arrival after request deadline | Pair receives zero allocation | Planned |
| JEX-024 | Request deadline already passed | Invalid request rejected | Planned |
| JEX-025 | Demand quantity greater than one | Exact quantity frontier matches oracle | Planned |
| JEX-026 | Offer minimum order quantity above one | Aggregate lot constraints hold | Authored |
| JEX-027 | Offer order increment greater than one | Aggregate lot congruence holds | Authored |
| JEX-028 | Two identical objective vectors from different allocations | One canonical assignment representative | Planned |
| JEX-029 | One allocation improves cost but worsens urgent-request shortage | Both non-dominated trade-offs remain | Authored |
| JEX-030 | One allocation strictly dominates another | Dominated objective is excluded | Authored |
| JEX-031 | Request-specific shortage has unlike physical unit | Shortage axes remain separate | Planned |
| JEX-032 | Requests contain different currencies | Reject mixed-currency optimization | Authored |
| JEX-033 | Unfilled quantities exceed u32 individually only via invalid report | No overflow or unchecked conversion | Planned |
| JEX-034 | Search node limit is zero | Invalid bound rejected | Planned |
| JEX-035 | Search node limit above hard maximum | Invalid bound rejected | Planned |
| JEX-036 | Search node counter reaches limit exactly | Completion status remains explicit | Planned |
| JEX-037 | Many pair edges with tight shared pool | Bounded search does not over-allocate | Planned |
| JEX-038 | Distinct capability profiles in same quantity unit | Only explicitly mapped edges can allocate | Planned |
| JEX-039 | Same cost, different request-level shortage vector | Both incomparable vectors retained | Authored |
| JEX-040 | Input order changes while manifest remains frozen | Output is deterministic and canonical | Authored |

## Qualification sequence

1. Exact-head compile, formatting, unit and integration tests.
2. Independent oracle for all binary matrices and several pool ceilings.
3. Expand oracle to quantities above one and lot increments.
4. Validate dominance/frontier results against literal expected vectors, not another call to production code.
5. Only then generate at least 10,000 deterministic micro-instances with pinned seed, generator version, corpus digest and source revision.
6. Record a completed search only when every required candidate was evaluated within the node limit.
