# South African Federated Fraud Signal Contract v1

**Status:** effects-disabled synthetic reference. This is not a fraud registry, risk model, SARB programme integration or legal decision service.

## Purpose and boundary

A fraud signal is a scoped, source-bound assertion, not an identity fact or a legal determination.

`signal received != allegation verified != account controlled by bad actor != legal finding != payment decision`

The receiver must retain its own decision authority. A signal may justify proportionate review or step-up authentication under an applicable participant policy; it must not create a permanent adverse identity fact or automatically deny essential payment access on its own.

## South African policy context

SARB's 2025 PEM Position Paper proposes a future NPU baseline fraud capability involving participant reporting of suspected and actual fraud and transactional fraud scoring. SARB's current PEM page says an insights capability and fraud framework will be developed. These sources describe programme direction, not a live national fraud API or current authority to share individual customer data.

The February 2026 BIS CPMI/PIE fraud report identifies data fragmentation and privacy/regulatory differences as barriers to cross-network fraud intelligence. The FPS interlinking report identifies common taxonomies and governance for lawful information exchange as important design considerations.

## Paradigm: federated evidence, not a global blacklist

The contract supports participant-to-participant or brokered query/response with explicit source, purpose, audience and expiry. It does not require copying raw customer histories into one central store.

Use provider-issued opaque subject tokens or purpose-scoped keyed commitments. A public hash of a phone number, bank account or national ID is not anonymisation. Never expose whether an arbitrary identifier exists through an unauthenticated query.

## Typed status and evidence

Signals may be:
- `suspected`;
- `corroborated` (requires independent source roots and participants);
- `confirmed_by_source` (a source's own final case disposition, not a universal/legal finding);
- `contested`;
- `under_review`;
- `corrected`;
- `withdrawn`;
- `expired`; or
- `unknown`.

Copies derived from one root report are not independent corroboration. A downstream participant copying a report does not create a new independent source.

## Purpose and disclosure

The synthetic profile permits only `payment_fraud_prevention` and the audience `participating_psp_fraud_team`. Disclosures are minimised and exclude raw account, phone, national ID and unrestricted history fields. Signals cannot be reused for credit scoring, marketing, unrelated eligibility or requester disclosure.

## Correction, appeal and expiry

Every status change is append-only. A correction/withdrawal supersedes the prior assertion without erasing history. Receivers must propagate status and stop presenting withdrawn, corrected, expired or otherwise stale evidence as current. Appeals remain visible as submitted/under review until an explicit outcome exists.

A receiver's prior decision remains auditable but may need reassessment after correction. The source remains accountable for the accuracy and timely updating of its signal.

## Benchmark

Hard gates:
- no suspected-to-confirmed promotion without source-authorised final evidence;
- no duplicate downstream reports counted as independent corroboration;
- no wrong source, audience, subject, jurisdiction or purpose;
- no expired, withdrawn or corrected assertion presented as current;
- no raw PII or public low-entropy identifier hash in shared output;
- no out-of-purpose disclosure or identifier-enumeration oracle;
- no permanent automatic denial based solely on one signal;
- all signal changes, corrections, withdrawals, appeals and expiries replay deterministically.

Outcome measurements should include false-positive and confirmed-signal rates on adjudicated matched cases, correction propagation time, stale exposure duration, appeal reversal, privacy leakage, unnecessary disclosure, wrongful denial, user friction and participant coverage. Keep suspected, corroborated, source-confirmed, contested, corrected and overturned cases separate. A high block rate is not success.

## Claim ceiling

The corpus is synthetic. A passing evaluator establishes only the typed provenance, purpose and lifecycle semantics of the fixture contract. It proves no fraud-detection accuracy, lawful sharing basis, nationwide interoperability, customer outcome, participant adoption or SARB endorsement.


## Exact event lineage and lifetime qualification

Every append-only history event now binds the signal version and SHA-256 content commitment it consumed. A status/correction/withdrawal update that changes the signal's content must increment the version and use a new digest. Same-version digest substitution, skipped versions, a final event that does not bind the current signal version, or an event sequence that moves backwards in UTC time is rejected.

Participant profiles declare a maximum signal TTL. The Rust evaluator parses UTC timestamps, requires observation before recording and expiry after observation, and rejects a lifetime beyond the profile maximum. Expired, corrected, withdrawn or unknown signals cannot be disclosed as current. The future-clock-skew case is represented as an untrusted quarantined event, not an accepted future event.

The 33-case corpus includes positive controls for independently corroborated signals and a source's own final case disposition, while keeping the latter explicitly a source assertion rather than a legal finding. The negative-mutation suite now contains 15 controls.


## 2026-11-04 recipient-side correction propagation extension

A source-side correction is not proof that a receiving institution has invalidated its cached copy. The corpus now separately records historical disclosures and one correction-propagation obligation per previous recipient.

### Legal profile distinctions

- South African POPIA section 24(3) links recipient notification to a correction that affects decisions already taken or to be taken and to what is reasonably practicable. Section 24(4) concerns notification to the data subject who requested correction.
- The 2025 POPIA Regulations' 30-day provision concerns written notification to the data subject of action taken on a correction/deletion request. It is **not** treated as a 30-day inter-participant cache-invalidation deadline.
- GDPR Article 19 is a separate EU comparator with its own impossibility/disproportionate-effort exception. It is not applied to a South African profile.
- A cross-border profile without a qualified jurisdictional rule remains unknown and cannot claim an operational legal notification outcome.

### Technical proof separation

`delivered != acknowledged != cached version invalidated != prior decision reassessed`

Each propagation obligation binds the recipient, historical version/digest, current version/digest, decision-impact assessment, practicability assessment, status, timestamps and receipt evidence. Pending, delivered or acknowledged-but-not-invalidated cases remain unresolved for operational purposes. A documented exception or reasoned "no decision impact" disposition is explicit; neither is silently treated as a successful invalidation.

The synthetic recipient-propagation profile does not establish a universal propagation SLA. Any measured p50/p95/p99 target must be set by the actual participant agreement and applicable law/profile.


## Historical recipient inventory completeness

A propagation report is not complete merely because every currently listed recipient has acknowledged. Before a corrected, withdrawn or expired signal can be reported as fully propagated, the source must qualify the historical disclosure inventory itself.

Each case now declares whether its prior-recipient inventory is `complete`, `incomplete`, `unknown` or `not_applicable`, together with a count, evidence reference and verification timestamp. A changed signal with an unknown or incomplete recipient inventory remains quarantined: the system cannot claim that every prior recipient was notified or invalidated. A complete zero-recipient inventory is different from a missing inventory and must be supported by its own evidence.
