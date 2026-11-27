# Economic OS temporal evidence contract v0.1

**Status:** typed temporal assertion vocabulary, synthetic fixture, Draft 2020-12 schema and public adversarial corpus. This is a research contract, not an authenticated clock service, source adapter, bitemporal database or production policy engine.

## 1. Architectural decision

Do not put one unqualified timestamp on an economic fact and expect it to answer every temporal question. The Economic OS needs separate assertions for:

- when an event is claimed to have occurred;
- when a state or right was valid in the domain;
- when a source system recorded the assertion;
- when a receiving system observed it;
- when a particular verifier knew and accepted it;
- the sequence/frontier of the owner journal that contains it.

The first five are temporal dimensions. The last is a logical owner-local order and is not a time value.

The versioned envelope is in `docs/economic/temporal/schemas/temporal-evidence-envelope.v1.schema.json`. A synthetic sample is in `fixtures/temporal-evidence-envelope.v0.1.json`. The 36-case public development corpus is in `benchmarks/temporal-evidence-adversarial-corpus.v0.1.json`.

## 2. Typed temporal dimensions

| Dimension | Question answered | What it must not be used for |
|---|---|---|
| OCCURRED_AT | When does the source assert the event happened? | Source-record commit, ingestion, verifier knowledge or causal order |
| VALID_FROM | From when was a domain fact/state/right valid? | When a system first learned it |
| VALID_TO | Until when was that fact/state/right valid? | An arbitrary max timestamp invented for an open interval |
| SOURCE_RECORDED_AT | When did the source system record/commit the assertion? | Event occurrence time |
| RECEIVER_OBSERVED_AT | When did this receiving system first observe the record? | Source event time or legal effective time |
| VERIFIER_KNOWLEDGE_AT | When did a specific verifier/profile accept the evidence into its qualified knowledge frontier? | A universal timestamp shared by every participant |

Every temporal assertion has its own time value, precision, quality, source/clock profile, source authority reference, evidence reference and content digest. Where relevant, an uncertainty window records earliest/latest plausible values. A time string being valid RFC 3339 does not authenticate its clock or authority.

Owner-local sequence, source-chain predecessor and checkpoint frontiers remain separately typed non-time fields. They are not converted into epoch timestamps.

## 3. Industry patterns to compose

### OpenTelemetry: event time versus observed time

The OpenTelemetry Logs Data Model defines Timestamp as when an event occurred, measured by the origin clock, and ObservedTimestamp as when the collection system observed it. Its Timestamp is optional when the source time is unknown. This is a strong interoperability precedent for preserving source occurrence time and collection observation time separately.

Official specification: https://github.com/open-telemetry/opentelemetry-specification/blob/main/specification/logs/data-model.md

Economic OS refinement: add source-recorded time and verifier-knowledge time as distinct, purpose-bound assertions. When converting to a format with only one time field, do not blindly use observed time as an economic occurrence time. A missing occurrence-time assertion should hold the event from an OCEL projection that requires event time.

### SQL Server: system-versioned history

SQL Server system-versioned temporal tables retain row history and database-managed period timestamps, allowing point-in-time reconstruction of what the database stored. The database period is based on the beginning of the transaction and captures system history; it does not independently establish when the underlying external event happened or when another verifier learned of it.

Official documentation: https://learn.microsoft.com/en-us/sql/relational-databases/tables/temporal-tables?view=sql-server-ver17

Economic OS refinement: pair system/record-time history with source occurrence time, domain valid-time and verifier-specific knowledge frontiers. This allows the query "what was valid at V according to evidence this verifier knew by K?" rather than only "what row did one database contain at system time T?"

### Mycelix composition

- FIN-POLICY-EXEC-001 / #5792 owns bitemporal policy decision semantics. Reuse its policy revision and historical replay model.
- FIN-EFFECT owns stable obligation and effect identities. Temporal assertions attach to those identities; they do not create replacement IDs.
- SAP causal frontier PR #6653 preserves explicit owner-local order. Identical timestamps do not add causal edges, and a timestamp sort does not resolve a distributed partial order.
- OCEL 2.0 JSON is a derived analytical projection. Its event time requires qualified occurrence-time evidence; source-record, receiver-observed and verifier-knowledge assertions remain in a provenance sidecar.

## 4. Bitemporal query semantics

A historical query is a pair:

- `valid_at = V`: which facts were valid in the domain at V?
- `known_at = K`: which assertions had this named verifier accepted by K?

The result is not necessarily today's view of what is believed to have been valid at V. It is the view that could have been produced from the verifier's qualified evidence frontier at K.

Example: a grant revocation is asserted to be valid from 18 November, recorded by its source on 20 November, ingested at 20 November 08:00:03 and accepted by a verifier at 08:00:20. A query about valid time 19 November with knowledge time 19 November must not silently include the revocation that the verifier did not yet know. A query with the same valid time and knowledge time 21 November can include it, subject to authority and evidence qualification.

Corrections are new immutable assertions with a supersession/correction reference. They do not rewrite the old timestamp, delete the old assertion or retroactively alter a query capped at an earlier knowledge frontier.

## 5. Valid-time intervals

When paired, VALID_FROM and VALID_TO describe a half-open interval `[from, to)`. An assertion set must reject a valid-to earlier than valid-from. An open-ended validity interval is represented by absence of a VALID_TO assertion plus an explicit open-ended interval policy; do not invent 9999-12-31 or another sentinel unless a pinned interoperability profile explicitly requires it.

Temporal overlap and interval closure are domain semantics. Two claims with overlapping or contradictory intervals may require conflict review; do not silently choose the latest arrival.

## 6. Clock quality and uncertainty

Clock profile and time quality are separate from legal/owner authority. A signature authenticates a signer and signed bytes only under the verified key/credential chain; it does not automatically prove the signer is authoritative for a time dimension or that the clock was synchronized.

Preserve:
- source clock profile and timezone/offset as received;
- precision and uncertainty interval;
- clock synchronization/calibration evidence where available;
- the authority scope of the source;
- the original time value and any normalized representation separately.

Do not silently round precision, normalize away offset evidence, or select earliest/latest among conflicting sources. Where uncertainty intervals overlap, record that fact without inventing an ordering. Where authoritative intervals are disjoint, hold as conflict.

## 7. Corrections, revocations and late arrivals

A late event can have an occurrence time earlier than the current owner frontier. It must be reconciled against source authority, explicit causal references and owner checkpoint state. Its arrival must not rewrite the owner chain or create a false timestamp order.

A revocation may be effective before it was recorded or known. Preserve all of:
- effective/valid time;
- source record time;
- receiver observation time;
- verifier knowledge time;
- authority and revocation evidence.

Historical as-known queries must remain byte/digest reproducible for frozen input and a fixed knowledge frontier, even when newer assertions later arrive.

## 8. Benchmark programme

The public 36-case corpus is a development set, not hidden evaluation. Targets, not achieved results:

| Dimension | Target |
|---|---:|
| Record/observed/knowledge time promoted to occurrence time | 0 |
| Owner sequence converted to time | 0 |
| Historical replay leakage across knowledge frontier | 0 |
| Silent correction/supersession overwrite | 0 |
| Acceptance of unresolved temporal conflict | 0 |
| Deterministic replay of frozen evidence/policy revisions | 100% |
| Explicit classification of missing/uncertain/contested cases | 100% |

Measure:
- ingestion-to-knowledge lag p50/p95/p99;
- late-arrival reconciliation latency;
- bitemporal query latency and storage/read amplification;
- correction and revocation propagation cost;
- temporal uncertainty coverage/calibration by source-clock profile;
- false ordering and false historical-knowledge promotion rates;
- operator reconciliation effort per 10,000 temporal assertions.

Matched baselines:
A. one timestamp and current-state-only view;
B. event-time + observed-time telemetry model (OpenTelemetry-style);
C. system-versioned row history (SQL Server temporal-table-style);
D. Economic OS occurrence/validity/record/observation/knowledge assertions plus owner frontiers and bitemporal policy replay.

Use identical frozen records, fault schedules and resource budgets. Hidden expected outcomes must not be visible to candidate logic. Do not treat candidate/LLM agreement as the primary oracle.

## 9. Claim ceiling

The current envelope, fixture and corpus do not establish source-clock correctness, legal effect time, authority, DHT completeness, settlement finality, production temporal consistency or audit admissibility. A verifier-known timestamp is specific to a verifier/profile and must be anchored to an independently qualified knowledge frontier. All temporal claims remain purpose-bound and evidence-scoped.
