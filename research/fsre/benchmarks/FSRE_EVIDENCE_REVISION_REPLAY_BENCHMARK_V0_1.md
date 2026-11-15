# FSRE evidence revision and historical replay benchmark v0.1

**Status:** protocol and synthetic known-answer cases. No field deployment or SOTA performance claim is made.

## 1. Industry practice comparators

### ONS: revisions and corrections are different events

The Office for National Statistics distinguishes revisions that incorporate improved methods, additional sources or data unavailable at initial publication from corrections made to address mistakes discovered after publication. It also explains the trade-off between timely provisional estimates and later accuracy:
https://www.ons.gov.uk/methodology/methodologytopicsandstatisticalconcepts/revisions/guidetostatisticalrevisions

**Benchmark implication:** a new value is not automatically a correction. Preserve the revision reason, the old value, the new value and the date the change became knowable. Test methodology changes separately from source/measurement corrections.

### DCMS 2026: scheduled revisions and transparent provisional data

DCMS's 11 June 2026 policy distinguishes scheduled revisions (including new data that changes previous periods) from corrections of errors, and says expected revisions should be handled transparently:
https://www.gov.uk/government/publications/standards-for-official-statistics-at-dcms/revisions-and-corrections-policy

**Benchmark implication:** a provisional status and planned revision schedule must be explicit. An as-of replay must never silently substitute a later value into an earlier decision.

### Statistics Canada 2026: temporal comparability

Statistics Canada's 2026 time-series guidance treats comparability as more than the timestamp of a row: the concepts, target population and measurement methods must remain comparable over time. It recommends parallel runs to assess discontinuities from redesigns:
https://www150.statcan.gc.ca/n1/pub/19-20-0001/192000012026004-eng.htm

**Benchmark implication:** methodology and population-boundary changes need separate impact measures. A technically valid replacement is not necessarily comparable to its predecessor.

These agencies are methodological comparators; FSRE is not claiming to replace official statistical governance or survey methods.

## 2. What FSRE adds

Existing statistical revisions policies describe governance and communication. FSRE's proposed contribution is an executable, evidence-level revision graph tied to decision episodes:

- immutable record versions and predecessor/successor relationships;
- separate observation, availability, valid-time and supersession-known times;
- replay of the version knowable at an exact decision cutoff;
- strict no-resurrection when an old record is superseded but its replacement is late or inadmissible;
- machine-readable explanation of which claims changed;
- propagation of changed claims to population denominators, partial-identification bounds and robust-safe actions.

This is a narrower technical claim than "better statistics": it is reproducible decision-vintage and downstream impact accounting.

## 3. Frozen adversarial scenarios

1. **Late correction:** an inspection is available at t=1000; a correction arrives at t=2000 and backdates its valid period. Replay at t=1500 must use the original version; replay at t=2500 must use the correction.
2. **Known supersession, late replacement:** source withdraws/supersedes old evidence at t=1500; replacement becomes available at t=2100. Replay at t=1800 must mark the old record superseded and the claim not estimable. Old evidence must not be resurrected.
3. **Late-known supersession:** replacement row is in the complete archive, but the relationship becomes known at t=2100. Replay at t=2000 must not apply the future relationship.
4. **Revision chain:** v3 supersedes v2, which supersedes v1. At a cutoff where both links are known, only v3 can support the current claim; all three versions remain present in the audit history.
5. **Fork:** two active successors claim to replace one predecessor. E0 must reject rather than choose the newest by row order.
6. **Cycle/dangling/cross-claim:** malformed revision lineage is rejected.
7. **Methodology change:** distinguish it from an error correction and trigger comparability review.
8. **Reclassification/boundary change:** a revision changes which service boundary or population stratum a record belongs to; denominators are recomputed under both vintages.
9. **Non-impacting correction:** the evidence value changes but the robust-safe action set is unchanged.
10. **Decision-flipping correction:** a corrected measurement widens the partial-identification interval enough that a formerly safe intervention is no longer robustly safe.

## 4. Metrics

### Replay and temporal correctness
- Historical replay mismatch rate against known-truth event histories.
- Future-known revision leakage rate.
- Superseded predecessor double-count rate.
- Silent resurrection rate.
- Supersession graph validation coverage.
- Reproducibility under input-row permutation.

### Revision-impact quality
- Changed-claim detection precision and recall.
- False revision-alert rate.
- Unchanged-decision false-alert rate.
- Decision-flip recall where ground truth changes the robust-safe set.
- Decision-stability preservation where the revision should not change the safe set.
- Re-review obligation latency and time-to-resolved-review (when operational data exists).

### Downstream integrity
- Population denominator delta by unit and service boundary.
- Partial-identification interval delta and width.
- Robust-safe set precision/recall.
- False-safe intervention count.
- Correct abstention rate.
- Realized service outcome delta when field evidence is available.

### Systems performance
- Replay graph nodes/edges.
- p50/p95 replay and delta-generation latency.
- Peak memory and I/O.
- Revision-chain length and fan-out.
- 10K, 1M and 10M evidence-record test points.
- Exact output determinism and input/output digests.

## 5. Matched baselines

- Mutable latest-row database query (intentionally vulnerable historical baseline).
- Bitemporal replay without explicit supersession edges.
- Revision-aware FSRE as-known replay.
- Independent revision-aware oracle.
- Manual revision register/runbook baseline for the downstream notification/review step.

Do not compare runtime unless all baselines use the same frozen event history, record ordering, hardware and replay cutoffs. Do not equate a fast database query with decision integrity.

## 6. Proposed targets

Research targets, not achieved:
- 100% exact replay parity across the frozen synthetic correction corpus.
- 0 future-known revision leakage.
- 0 predecessor/successor double counting after supersession is known.
- 0 silent resurrection if the replacement is late, stale, expired or unknown.
- 100% rejection of E0 revision forks, cycles, dangling and cross-claim links.
- 100% row-permutation determinism.
- 100% correct known-truth classification of changed versus unchanged robust-safe action sets on the frozen corpus.
- Publish precision/recall and false-alert rate for any revision-impact detector.
- Report p50/p95 latency and memory at each scale; do not claim superiority without matched runs.

## 7. Integration path

- #6192/#6201: bitemporal identity and service/asset namespaces.
- #6273/#6274: evidence revision graph and immutable as-of replay.
- #6247/#6249: typed decision episode and proof obligations.
- #6253: assurance-obligation acquisition planner.
- #6255: exact partial-identification LP certificates.
- #6207: integrated Decision Integrity Benchmark.

The key outcome is not that the current record is always "right". It is that FSRE can reproduce what was knowable when a decision was made, disclose what changed later, and identify which decisions need review without rewriting history.
