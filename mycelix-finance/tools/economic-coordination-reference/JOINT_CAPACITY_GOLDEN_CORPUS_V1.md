# Joint Capacity Allocation — golden corpus registry v1

**Status:** stable fixture-ID registry; not all fixtures are implemented.  
**Tracking issue:** [PROC-MULTIREQ-001 #5144](https://github.com/Luminous-Dynamics/mycelix/issues/5144).  
**Implementation:** [joint_capacity.rs](https://github.com/Luminous-Dynamics/mycelix/blob/feat/procurement-joint-capacity-validation-v1/mycelix-finance/tools/economic-coordination-reference/src/joint_capacity.rs).

The current implementation has 19 authored module tests. Test names are included where they cover a row; the registry's remaining cases are planned. No row is declared passing until exact-head CI has executed it. “LiveDomain” in a synthetic fixture tests branch behavior only and does not authenticate real evidence.

| Fixture ID | Scenario | Expected invariant | Status |
|---|---|---|---|
| JCV-001 | Valid two-request allocation across distinct pools | Per-request cost and service plus pool/unit/currency summaries reconcile; line witnesses identify mapping and pool snapshot | Authored |
| JCV-002 | Two offers reference one pool; combined quantity exceeds pool ceiling | Joint allocation rejected with pool-capacity error | Authored |
| JCV-003 | Two requests try to claim one last shared unit through separate offers | Pool capacity is counted once; second claim rejected | Authored |
| JCV-004 | Request is only partially filled | Unfilled quantity visible; fully-served remains false | Authored |
| JCV-005 | Capacity snapshot expired at evaluation time | Fail closed | Authored |
| JCV-006 | Verified synthetic capacity | Not promoted to live capacity | Authored |
| JCV-007 | Capability mapping expired | Assignment rejected | Authored |
| JCV-008 | Request budget just below assigned cost | Assignment rejected | Authored |
| JCV-009 | Duplicate request/offer assignment row | Non-canonical duplicate rejected | Authored |
| JCV-010 | Offer currency differs from request currency | Assignment rejected | Authored |
| JCV-011 | Aggregated order quantity violates min/increment | Invalid lot rejected; compliant aggregate accepted | Authored |
| JCV-012 | Requests/offers/mappings/assignments permuted | Canonical report struct-equal, including ordered line-level witnesses | Authored |
| JCV-013 | Offer arrives after request deadline | Assignment rejected | Authored |
| JCV-014 | Duplicate request ID | Input rejected | Planned |
| JCV-015 | Duplicate offer ID | Input rejected | Planned |
| JCV-016 | Duplicate pool ID with conflicting ceilings | Input rejected, never last-write-wins | Planned |
| JCV-017 | Offer references missing pool | Input rejected | Planned |
| JCV-018 | Pool unit differs from offer unit | Input rejected | Planned |
| JCV-019 | Request quantity exceeds aggregate assigned quantity by one | Exact demand upper bound | Planned |
| JCV-020 | Request assigned above demand at u32 boundary | Rejected without wraparound | Planned |
| JCV-021 | Offer aggregate exceeds offer-local cap while pool has slack | Rejected by offer limit | Planned |
| JCV-022 | Offer-local cap is larger than shared pool cap | Pool still remains binding | Planned |
| JCV-023 | Missing capability mapping on one assignment | Assignment rejected | Planned |
| JCV-024 | Mapping points to unknown request or offer | Input rejected | Planned |
| JCV-025 | Mapping digest/profile does not match request and offer revisions | Input rejected | Planned |
| JCV-026 | Mapping status unknown, stale or contested | Assignment rejected | Planned |
| JCV-027 | Mapping evidence observed after evaluation cutoff | Assignment rejected | Planned |
| JCV-028 | Offer qualification status unknown or contested | Input rejected | Planned |
| JCV-029 | Offer evidence is from synthetic simulation | Not treated as live supply | Planned |
| JCV-030 | Offer evidence observed after evaluation cutoff | Input rejected | Planned |
| JCV-031 | Offer evidence expired at cutoff | Input rejected | Planned |
| JCV-032 | Capacity evidence is from synthetic simulation | Not treated as live capacity | Authored |
| JCV-033 | Request and offer units differ | Assignment rejected | Planned |
| JCV-034 | Requests use identical units but different capability profiles | Unit totals remain separated by capability/profile | Planned |
| JCV-035 | Requests use multiple currencies | Costs are grouped by currency, never summed as one scalar | Planned |
| JCV-036 | Request budget is exactly equal to allocated unit cost | Accepted at inclusive budget boundary | Planned |
| JCV-037 | Unit-cost multiplication overflows u64 | Fail closed with arithmetic error | Planned |
| JCV-038 | Aggregating request or pool quantities overflows u64 | Fail closed with arithmetic error | Planned |
| JCV-039 | Empty manifest digest or required identity | Input rejected | Planned |
| JCV-040 | All assignment rows empty for valid demand | Deterministic all-unfilled report; no synthetic success | Planned |

| JCV-041 | Exhaustive 2-request x 2-offer binary assignment matrix at three shared-pool ceilings | 48 cases agree with test-local independent demand/offer/pool feasibility oracle | Authored |
| JCV-042 | Unit-cost times quantity exceeds u64 | Checked arithmetic error, never a wrapped passing report | Authored |
| JCV-043 | Same literal unit but different capability/profile | Quantity totals stay separated by capability/profile/unit | Authored |
| JCV-044 | Empty request set with otherwise empty manifest | Invalid manifest rejected | Authored |

| JCV-045 | Request deadline precedes evaluation cutoff | Prospective allocation rejected as expired | Authored |
| JCV-046 | Accepted allocation witness source audit | Source URI/digest, qualification receipt, evidence status/origin and validity times retained | Authored |

## Qualification rules

- Preserve every fixture ID as implementation evolves.
- Mark a row Authored only when an executable test actually covers its invariant.
- Mark a row Passing only in a revision-specific qualification record after exact-head CI runs.
- The checker’s output is not its own independent oracle. Test-local expectations and the small-instance exhaustive oracle must be independently implemented.
- Once small-instance oracle qualification is complete, run at least 10,000 deterministic generated cases with pinned generator, seed, manifest, corpus digest and revision. Counterexamples become named fixtures.
- Keep the verified-synthetic negative control, stale-evidence controls and deliberate shared-pool over-allocation cases permanently.
