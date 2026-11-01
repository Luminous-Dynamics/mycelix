# GCPN Bitemporal Warning-Evidence Graph Lifecycle Audit v0.1

**Status:** research specification and machine-readable contract; not an executable conformance result.  
**Parent issue:** #5646.  
**Stacked on:** integrated GCPN evaluation v0.6, branch `research/gcpn-integrated-evaluation-v0-6`.  
**Scope:** offline synthetic civilian-warning evidence, provenance, temporal reconstruction and CAP lifecycle conformance.

## 1. Research question

Can an independent auditor reconstruct what a warning system knew, asserted, published, delivered and later corrected, without silently losing contradictory evidence or rewriting history?

The current GCPN work has separate tests for authority provenance, forecast records, last-mile receipts, accessibility, comprehension, action evidence and lifecycle cost. The next capability should test the **relationships between those records through time**.

The hard cases are not just missing rows. They are late-arriving observations, changed forecasts, message updates and cancellations, duplicated receipts, conflicting identifiers, expired or revoked authority, contradictory but authentic sources, and later corrections to previously available evidence.

The benchmark is an offline synthetic conformance programme. It does not create, authorize, publish or dispatch warnings.

## 2. Paradigm shift: immutable evidence events and bitemporal reconstruction

Replace mutable "current status" records with immutable, typed evidence objects and append-only relations. A correction creates a new object and an explicit relation to the earlier object. It never edits the earlier evidence bytes in place.

The evaluator must answer two separate questions:

1. **Current view:** What is the best-supported state of the graph using all evidence currently recorded?
2. **Historical as-of view:** What evidence had been recorded and was available by transaction-time cutoff T, under the rules and authority state then in force?

This requires two independent notions of time.

- **Valid/event time:** when a source says an event, observation, forecast-valid period, authority assertion, message or action applies.
- **Transaction/knowledge time:** when that record entered the auditable system and became available to its users.

Do not infer transaction time from source timestamps. A report created at 09:00 but received at 11:30 cannot appear in an 10:00 historical knowledge view. A revocation learned today may affect today's verification; it must not be silently backdated into what an auditor could have known yesterday.

The benchmark should make the graph reconstructible from an ordered event log, not only from a precomputed final snapshot.

## 3. Evidence object model

Use a small, typed core inspired by W3C PROV-DM/PROV-N. PROV distinguishes entities, activities, agents and relations such as generation, use, derivation and attribution. It is a provenance model, not a mechanism that proves an observation is physically correct or a person/institution has legal warning authority.

### 3.1 Node classes

| Class | Examples | Required identity/evidence |
|---|---|---|
| Entity | source observation, forecast product, authority assertion, CAP message version, delivery receipt, accessible rendering, comprehension observation, action record, outcome evaluation | Stable ID, type, content digest or explicit non-content reference, valid-time fields, transaction-time fields, source/provenance |
| Activity | observe, ingest, forecast, verify, authorize, publish, transform, transmit, render, assess, act, evaluate, revoke | Stable ID, activity type, start/end or point time, responsible agent, used/generated entity references |
| Agent | source institution, forecast provider, warning authority, relay/channel provider, reviewer, independent evaluator | Stable ID, declared role and jurisdiction/scope, credential reference and validity/revocation evidence where applicable |

### 3.2 Typed relations

The initial vocabulary should include:

- `used`: activity used an entity;
- `wasGeneratedBy`: entity was produced by an activity;
- `wasAssociatedWith`: activity was performed by an agent;
- `wasAttributedTo`: entity is attributed to an agent;
- `wasDerivedFrom`: entity is derived from another entity;
- `supports` and `contradicts`: evidence relations, with explicit status and rationale reference;
- `supersedes`: later version replaces an earlier version for a declared scope;
- `revokes`: authorized revocation of an assertion, credential or authority grant;
- `references`: CAP/reference link that does not itself imply supersession;
- `deliveredAs`: receipt is tied to an exact message version/digest;
- `renderedAs`: accessible rendering is tied to an exact message version and language/format;
- `assessedBy`: a comprehension/outcome observation is linked to a declared assessment activity;
- `actionEvidenceFor`: action evidence refers to a warning/message version;
- `evaluatedBy`: outcome claim is linked to an independent evaluation activity/agent.

The final schema must document direction, endpoint types, cardinality, time semantics and whether cycles are allowed for every relation. A generic untyped `parent_id` is not sufficient.

### 3.3 Evidence state is not truth

Keep independent state dimensions. At minimum:

- integrity: unchecked, digest-valid, digest-mismatch;
- authenticity: unknown, verified, failed, revoked-key;
- authority: not-asserted, in-scope, out-of-scope, expired, revoked, disputed;
- measurement/evidence validity: unknown, valid-under-policy, invalid, stale, incomplete;
- corroboration: uncorroborated, independent-support, dependent-support, contradiction-present;
- lifecycle: current, superseded, cancelled, expired, withdrawn;
- privacy: public-safe, restricted, prohibited-leak.

These are not one ordered confidence ladder. For example, an authentic observation may be inaccurate; an authority assertion may be signed but out of scope; a digest-valid message may contain a false statement; a stale observation may remain historically relevant.

Missing data must remain explicitly missing/unknown. It must never be silently converted into zero, false, absent risk, successful verification or positive evidence.

## 4. Bitemporal contract

Every node and material relation needs enough metadata to reconstruct both its claim time and its recorded/knowledge time.

Recommended common fields:

- `valid_from`, `valid_to`: interval during which the assertion is said to apply;
- `recorded_at`: trusted ingestion/ledger transaction time;
- `source_created_at`: time stated by the source (not trusted as transaction time);
- `observed_at`: event/measurement occurrence time, where applicable;
- `superseded_at`: transaction time when the successor relation became known;
- `revocation_effective_at`: stated effective time of revocation;
- `revocation_recorded_at`: time revocation evidence entered the system;
- `policy_version`: policy used to interpret scope/freshness/validity.

Do not require irrelevant fields for every node type: use typed schemas and explicit nullable applicability. Never substitute one clock for another.

### As-of query semantics

Given a graph event log E and transaction-time cutoff T, reconstruct only objects/relations whose transaction-time records were available by T. Preserve valid-time intervals as attributes; apply the declared policy version that was available at T. Later knowledge may change the current view but cannot mutate the prior as-of result.

The evaluator must test:
- record arrives after its source-created time;
- message update arrives after the first message;
- revocation is effective before it is learned;
- revocation is effective after it is learned;
- an authority or credential expires without an explicit revocation;
- a historical query is repeated after later updates have been appended.

A historical as-of query is about what was recorded/available, not an assertion that the earlier view was objectively correct.

## 5. CAP 1.2 lifecycle boundary

CAP 1.2, standardized in ITU-T X.1303 bis, remains the interoperable message object. GCPN is a separate evidence/provenance sidecar. Do not invent a competing alert XML format or change CAP field meaning.

Preserve and validate CAP fields relevant to lifecycle and identity, including `sender`, `identifier`, `sent`, `status`, `msgType`, `references`, `expires`, `scope` and message `info` content. Retain the original message bytes and their cryptographic digest. Do not create a digest by rewriting or normalizing XML unless a separately specified canonicalization profile is adopted.

CAP message-type behavior to cover:
- **Alert:** initial alert message.
- **Update:** references one or more prior messages and provides updated information.
- **Cancel:** references one or more prior messages and cancels them.
- **Ack:** acknowledgement message; does not prove human comprehension or action.
- **Error:** error/rejection message; preserve its relationship to the relevant exchange.

Expiry and cancellation are different lifecycle facts. An expired alert is not automatically evidence that a cancellation message was sent. An update or cancellation without required references is a conformance failure. A successor must not mutate the bytes of the predecessor it references.

A receipt proves receipt of a particular transmitted message/version under the receipt's channel semantics. It does not prove that a person saw, understood, trusted or acted on it.

## 6. Cryptographic and source-independence semantics

Use SHA-256 or another explicitly versioned cryptographic digest for payload-integrity checks. Record the algorithm and exact digest input bytes. The existing FNV-1a fixture fingerprint may continue as a convenient non-cryptographic regression checksum; it is not an authenticity, tamper-resistance or signature mechanism.

Keep these questions separate:

1. Are the bytes the same as the bytes whose digest was recorded?
2. Was the message signed by a key that verifies under the declared trust policy?
3. Was the key/credential valid and unrevoked at the relevant transaction/valid time?
4. Did the agent have the declared authority for this hazard, territory, channel and time?
5. Is the underlying observation or forecast scientifically valid?
6. Are corroborating sources independent, or do they share an upstream provider, sensor, model or data feed?

The benchmark can test the declared synthetic rules. It cannot establish physical truth or confer legal authority.

## 7. Fail-visible invariants

1. **Exact duplicate:** same stable identity and exact digest is idempotent, while duplicate-receipt counts remain visible.
2. **Conflicting duplicate:** same identity with different bytes/digest remains a conflict; do not choose one silently.
3. **Missing lineage:** missing source, forecast or derivation links remain missing and fail any gate that requires them.
4. **Late evidence:** current assessment may change; earlier as-of views remain reproducible.
5. **Contradiction:** both claims and source lineage remain visible while dispute state is unresolved.
6. **Supersession:** successor identifies exact predecessor/version and scope; predecessor bytes remain immutable.
7. **Revocation:** preserve original signed assertion plus revocation evidence, effective time, recorded time, source and policy version.
8. **Freshness:** freshness windows are typed and policy-versioned by evidence class/authority; no global TTL is assumed.
9. **Authority:** signed/authentic is not equivalent to in-scope or authorized.
10. **Source independence:** multiple records from one upstream provider are not counted as independent sources.
11. **Graph integrity:** dangling, orphaned, malformed or forbidden-cycle edges are reported by expected reason.
12. **CAP lifecycle:** update/cancel references resolve to exact earlier message identifiers; the original message bytes remain unchanged.
13. **Receipt binding:** receipt points to an exact message version and digest, not merely a campaign or alert identifier.
14. **Rendering lineage:** translation/accessibility output points to the exact message version and transformation activity.
15. **No semantic shortcuts:** delivery does not imply comprehension; comprehension does not imply action; action does not imply causal impact.
16. **Privacy projection:** public exports exclude private recipient identity and restricted source content while maintaining a verifiable relation to the underlying protected record.
17. **Historical stability:** later arrivals, corrections, supersessions or revocations cannot rewrite prior as-of snapshots.

## 8. Frozen synthetic graph and mutation suite

Start with a tiny, human-auditable baseline graph containing at least two sources with deliberately disclosed independence, one forecast, one authority assertion, a CAP Alert and Update, delivery and accessible-rendering receipts, and distinct placeholder nodes for comprehension, action and outcome. These human-outcome nodes are synthetic fixtures, not observations about people.

Each negative control must mutate the fixture, recalculate the fixture digest, run the evaluator, and assert a specific expected diagnostic. Do not accept any arbitrary error as proof that the intended guard works.

Required mutation families:

| Family | Mutation | Expected diagnostic |
|---|---|---|
| Missing source | Remove source observation node | missing-source-lineage |
| Missing derivation | Remove forecast-to-observation edge | missing-derivation |
| Late arrival | Record source after event time | late-evidence-visible; historical-as-of-excludes |
| Exact duplicate | Repeat same ID and digest | idempotent-duplicate-counted |
| Conflicting duplicate | Reuse ID with changed digest | conflicting-identity |
| Tamper | Alter bytes without updating expected digest | digest-mismatch |
| Replay | Reintroduce an old valid message/receipt in a new context | replay-detected-or-review-required |
| Update without reference | Remove required prior-message reference | cap-update-reference-missing |
| Wrong update reference | Point to different message/version | cap-reference-mismatch |
| Cancel without reference | Remove required cancel reference | cap-cancel-reference-missing |
| Late supersession | Add successor after historical cutoff | current-view-updated; historical-view-stable |
| Contradiction | Add incompatible authenticated source claim | contradiction-preserved |
| Shared provider | Two apparent sources share upstream provider | independence-not-established |
| Revoked key | Use signature with revoked credential under selected time view | credential-revocation-visible |
| Out-of-scope authority | Change hazard/territory/time scope | authority-scope-failed |
| Expired assertion | Query after authority validity end | authority-expired |
| Stale evidence | Exceed policy freshness interval | stale-not-converted-to-invalid-or-false |
| Orphan | Remove referenced activity or agent | orphan-or-dangling-reference |
| Cycle | Insert cycle in relation class declared acyclic | forbidden-cycle |
| Privacy leak | Include recipient identifier in public projection | public-projection-privacy-failure |
| Receipt mismatch | Attach receipt to wrong message digest | receipt-message-digest-mismatch |
| Detached rendering | Bind translation to another message version | rendering-lineage-mismatch |
| Invalid inference | Add delivered-to-comprehended edge | forbidden-semantic-inference |
| Overwritten history | Mutate predecessor bytes when adding Update | predecessor-immutability-violation |

The evaluator should report all relevant diagnostics for a record when safe, but a test asserts its intended mutation reason explicitly. Fixture hashes pin exact bytes and schema version. A checksum mismatch is not itself the intended semantic failure if the scenario deliberately regenerates its expected fixture digest.

## 9. Metrics

Report each independently:

- entity/activity/agent counts by type;
- relation counts and typed-edge conformance;
- missing lineage, orphan, dangling-reference and forbidden-cycle counts;
- exact duplicate and conflicting identity counts;
- cryptographic digest mismatch and signature-verification diagnostics;
- replay detection and unresolved replay review count;
- bitemporal as-of snapshot correctness;
- late evidence and revocation visibility by cutoff;
- supersession predecessor immutability and lineage completeness;
- authority scope/expiry/revocation conformance;
- contradiction preservation and resolution status;
- source-independence classification accuracy;
- privacy violations in public projection;
- exact-message digest receipt match;
- rendering-to-message-version lineage match;
- invalid semantic-inference edge count;
- false accepts and false rejects by mutation family;
- deterministic replay, runtime and memory.

No single graph trust score is permitted. Structural validity, authenticity, authority, scientific validity, independence, privacy and human outcome are distinct claims. A graph can be perfectly well-formed while its underlying evidence is unverified or contradictory.

## 10. Relationship to the eight-gate GCPN evaluation

Do not change or renumber the eight v0.6 modules in this PR.

This graph benchmark starts as a separate package. A future ninth gate may be considered only after:
1. the standalone schema and lifecycle semantics have been implemented;
2. all mutation fixtures and negative controls are exact and independently reviewed;
3. bitemporal snapshots and cryptographic digest semantics have dedicated tests;
4. CAP lifecycle behavior is checked against the published standard;
5. privacy and authority boundaries have a documented review.

GCPN remains an evidence/provenance and conformance sidecar. It is not a warning authority, forecast producer, national MHEWS, CAP replacement, transport layer, emergency-response organization or a substitute for local community governance.

## 11. Evidence progression and human research boundary

Synthetic graph conformance is only the first stage. A responsible future sequence is:
1. synthetic schema and lifecycle conformance;
2. authorized, de-identified historical replay;
3. community co-design of message and accessible-rendering semantics;
4. independent ethics review and consented human-subject comprehension studies;
5. closed tabletop/continuity exercises under authority oversight;
6. prospective shadow evaluation without dispatch or operational decisions;
7. any operational integration only under the relevant national authority, legal agreement and independent assessment.

Synthetic delivery/comprehension/action/outcome nodes must not be reported as participant outcomes, successful actions, lives saved or causal effects.

## 12. Sources and standards alignment

- W3C, *PROV-DM: The PROV Data Model* (W3C Recommendation): https://www.w3.org/TR/prov-dm/
- W3C, *PROV-N: The Provenance Notation* (W3C Recommendation): https://www.w3.org/TR/prov-n/
- OASIS, *Common Alerting Protocol Version 1.2*: https://docs.oasis-open.org/emergency/cap/v1.2/CAP-v1.2.html
- ITU-T Recommendation X.1303 bis, *Common alerting protocol (CAP 1.2)*: https://www.itu.int/itu-t/recommendations/rec.aspx?lang=en&rec=12150
- UNDRR/WMO, *Global Status of Multi-Hazard Early Warning Systems 2025*: https://www.undrr.org/reports/global-status-mhews-2025

CAP defines a standardized all-hazard message format; it does not by itself prove delivery to a person, accessible representation, comprehension or action. W3C PROV supplies a general provenance vocabulary; it does not certify physical truth, legal authority, source independence or human outcomes. GCPN should complement these standards, not replace them.

## 13. Non-claims

This specification is not a live warning network, operational CAP gateway, alert authority, scientific source validator, cryptographic trust root, legal compliance certification, human-subject study or proof of lives saved. It contains no live alert dispatch, recipient tracking, emergency resource control, physical hazard verification, military or weapons capability. Cryptographic integrity is not physical truth; provenance completeness is not legal authorization; graph conformance is not causal impact.
