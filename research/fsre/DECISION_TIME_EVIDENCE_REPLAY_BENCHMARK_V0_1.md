# FSRE decision-time evidence replay benchmark v0.1

**Date:** 2026-11-14  
**Status:** Research and synthetic reference implementation. No production bitemporal store, source-authenticity guarantee, legal admissibility determination or field decision replay is claimed.

## 1. Research thesis

Infrastructure decisions are often audited against the evidence currently available, not the evidence that was actually available when the decision was made. This creates hindsight contamination: late-arriving evidence, retrospective corrections, later revocations and changed population/asset mappings can silently rewrite an earlier decision's apparent basis.

FSRE proposes an as-known-at-the-time evidence qualification layer. Its question is not merely "what is true now?" or "what was true at time t?" It asks:

> At decision time \(t_d\), which evidence versions had been observed, recorded, were valid in the represented world, remained unrevoked, and fell within the scope of the decision?

This extends established temporal-data methods; it does not invent bitemporal databases. Its research contribution is to apply multi-clock evidence eligibility and replay to cross-system infrastructure recovery claims, where an evidence version can be valid in the world but unavailable to the decision-maker, or recorded but outside its declared validity interval.

## 2. Temporal dimensions

Keep the following clocks separate. Do not collapse them into one timestamp field.

| Clock | Meaning | Qualification question |
|---|---|---|
| Valid time | When a fact applies in the represented world | Was the claim valid at decision time? |
| Observation time | When the source observed, measured or issued the evidence | Could the source have known this at decision time? |
| Transaction/record time | When the receiving system persisted the evidence version | Was it actually available to this decision process? |
| Decision time | The information frontier for the decision being replayed | Which versions are eligible as-of this instant? |
| Revocation time | When an issuer withdrew or revoked a claim/grant | Had revocation taken effect by decision time? |

For a submitted evidence version \(e\), the knowledge-frontier filter is:

\[
e.observed\_at \leq t_d \quad\land\quad e.recorded\_at \leq t_d
\]

After selecting the latest eligible revision, the valid-time and revocation conditions are:

\[
e.valid\_from \leq t_d < e.valid\_until
\quad\land\quad
(e.revoked\_at = \varnothing \;\lor\; e.revoked\_at > t_d)
\]

A version failing either knowledge-frontier condition is excluded from that replay, even when its claim is later known to have been true. A later correction does not rewrite the earlier snapshot. The reference uses half-open valid-time intervals \([from, until)\), so adjacent periods do not overlap at their shared boundary.

## 3. Existing industry capability and the actual gap

Temporal storage and point-in-time reconstruction are established capabilities.

- Microsoft SQL Server system-versioned temporal tables automatically retain row history and support point-in-time queries using system-maintained period columns. Microsoft describes these as database-managed system time; history represents when row versions existed in the database. This is a strong transaction-time baseline, but it does not by itself establish when an external operator observed a fact or the period in which that fact was true in the outside world. Documentation: https://learn.microsoft.com/en-us/sql/relational-databases/tables/temporal-tables?view=sql-server-ver17.
- PostgreSQL 19 documents application-time temporal keys and temporal foreign keys, but explicitly says system time is not currently supported natively. Application/valid time and database transaction history are separate capabilities. Documentation: https://www.postgresql.org/docs/19/ddl-temporal-tables.html.
- The SQL:2011/SQL:2023 temporal model distinguishes application/valid time from system/transaction time. The PostgreSQL project's SQL2011Temporal overview explains their orthogonality and the half-open interval convention: https://wiki.postgresql.org/wiki/SQL2011Temporal.
- A 2026 PRISMA-guided review documents variation in bitemporal terminology, modelling and implementation across literature and commercial systems: https://doi.org/10.1007/s42488-026-00162-x.

FSRE must not claim to outperform temporal databases at generic as-of queries. The testable gap is the decision evidence envelope across organisations and source systems: valid time, source observation, receiving-system record time, revocation, claim identity, owner provenance, scenario, scope and the decision's explicit information frontier remain linked.

## 4. Reference implementation boundary

The Rust reference at tools/fsre-decision-time-replay is a read-only evidence-version eligibility checker. It:
- reconstructs the latest revision for each evidence ID whose source observation and system record times are both at or before the decision cut-off;
- excludes later observations or records from the historical decision state;
- classifies the selected version as known-valid, not-yet-valid, expired, revoked or source-unknown;
- rejects duplicate revisions, changes to claim identity/subject/predicate/owner across revisions, backwards transaction time, invalid time intervals, and records persisted before their source observation;
- preserves owner and evidence references and reports the number of later revisions excluded.

It does not authenticate evidence, prove assertions true, establish that all required claims were submitted, decide whether an authority is legitimate, or authorise an operational action. Its ALL_SUBMITTED_EVIDENCE_VALID_AT_DECISION disposition applies only to submitted evidence IDs, not to an independently established complete claim universe.

## 5. Challenge families

The public synthetic suite should test at least:

1. Late ingestion: source observed before the decision but system recorded it after the decision. It is not available to the as-of replay.
2. Future observation: record stored early or staged in advance, but source observation occurs after the decision. It cannot be treated as observed evidence.
3. Retrospective correction: a later version corrects the fact; replay before the correction must preserve the prior known version.
4. Backdated validity: a correction recorded later asserts an earlier valid-from date; earlier decisions must not silently inherit it.
5. Revocation: a revocation known before decision excludes the evidence; a revocation learned later does not leak backward.
6. Expiry boundary: evidence ending exactly at decision time is expired under [from, until).
7. Not-yet-valid grant: evidence becomes valid only after the decision cut-off.
8. Claim identity drift: later revision changes the subject or predicate; reject or split the evidence identity rather than overwrite it.
9. Duplicate/out-of-order revisions: revisions repeat or transaction time moves backwards.
10. Clock inconsistency: transaction time precedes source observation; reject and record the clock-quality defect.
11. Unknown evidence: source state is unknown or a required evidence ID has no as-of version; abstain rather than infer.
12. Scenario/scope crossover: evidence from another scenario, asset, geography or decision authority must not qualify the current claim.

A second sealed challenge set must be authored from real, lawfully shareable operator event histories and adjudicated independently. Synthetic conformance is not field proof.

## 6. Matched baselines and ablations

Run the same frozen evidence stream and decision times through:
- B0: current-state query only;
- B1: valid-time-only query;
- B2: transaction/system-time-only temporal query (for example, SQL Server FOR SYSTEM_TIME AS OF);
- B3: application-time temporal constraints only (for example, PostgreSQL temporal keys/foreign keys);
- B4: bitemporal schema/query implementation;
- B5: event-sourcing/audit-log reconstruction used by the participating operator;
- B6: operator's manual historical decision reconstruction;
- B7: FSRE evidence envelope with valid, observed, recorded and revoked times;
- ablations removing observation time, transaction time, revocation, revision identity, scenario scope or evidence-owner provenance one at a time.

Use each product's supported semantics. Do not make an application-time database appear deficient for not providing system time, or treat a database history table as external source authenticity.

## 7. Metrics and proposed qualification gates

Deterministic conformance targets:
- zero future-version leakage into a decision-time replay;
- 100% exact selected-revision agreement with an independently authored reference;
- zero acceptance of a selected version outside its valid-time interval;
- zero acceptance of a revocation known effective by decision time;
- zero revision identity drift silently accepted;
- 100% preservation of selected source owner, evidence reference, revision and decision cut-off;
- unknown or missing as-of evidence produces explicit abstention;
- all timestamps use a declared time basis and precision.

For operator studies, report as-of reconstruction accuracy against independently adjudicated decision snapshots; future-leakage and invalid-evidence false-accept rates; false rejection and abstention coverage; lineage completeness; correction/revocation propagation latency; replay throughput and p50/p95/p99 latency at declared history sizes; operator reconstruction effort; storage/index overhead and retention cost; and clock-skew/late-arrival sensitivity.

Do not set a throughput target before benchmarking realistic histories, indexes, concurrency and retention. Correctness and absence of future leakage are hard gates; performance is measured against matched temporal database baselines.

## 8. Integration and safeguards

- Mycelix remains the owner of commitments, evidence lineage, identity, authority and revocation records.
- Sol-Atlas supplies spatial/asset identity and temporal boundary evidence.
- Symtropy supplies model snapshots and the validity domain for physical claims.
- Symthaea may investigate disagreements, but cannot rewrite historical source evidence.
- Service owners and accountable authorities define which claims are required for a decision; this evaluator cannot invent that set.
- Independent evaluators reproduce the historical evidence frontier and actual service outcomes.

Time replay does not make a claim true, legally admissible or operationally authorised. A cryptographic signature can support integrity and attribution; it does not by itself prove source accuracy, observation time, clock correctness or validity. Those remain separate qualification questions.

## 9. Falsification

Narrow or reject the FSRE claim if:
- operator source systems cannot provide trustworthy observation and transaction times;
- clock error is larger than the time precision required by the decision and cannot be bounded;
- historical versions are incomplete or mutable without an independent receipt;
- current temporal databases and operator processes reconstruct the same evidence frontier with equivalent fidelity and lower burden;
- the additional envelope does not reduce future leakage, invalid evidence acceptance or operator reconstruction effort;
- preserving decision-time evidence introduces unacceptable latency or operational delay.

The intended achievement is not a novel temporal database. It is a portable, replayable decision-evidence contract for infrastructure recovery, with independently testable historical knowledge boundaries.
