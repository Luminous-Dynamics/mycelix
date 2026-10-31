# NRVDEF Evidence Custody and Verification-Boundary Benchmark v0.1

**Status:** synthetic research specification and fixture; not a treaty protocol, inspection authority, or compliance system.  
**Parent:** NRVDEF issue #5588; benchmark issue #5592.  
**Data scope:** generic, aggregate, non-sensitive declarations and evidence-custody metadata only.

## 1. Objective

Build a reproducible test boundary around digital evidence used in risk-reduction and disarmament processes. The system may establish that a record is authentic, that a custodian received it, that a custody chain is internally complete, and that an authorized institution issued a decision. It cannot establish physical dismantlement from a signed declaration or a cryptographic receipt.

The benchmark deliberately contains no weapon design, facility geometry, material-accounting details, inspection routes, tamper-evasion methods, launch systems, or operational command information.

## 2. The fundamental epistemic separation

Never collapse these independent claims:

| Claim | What software can establish | What software cannot establish by itself |
|---|---|---|
| Record integrity | Bytes match a declared digest; signature validation result under a named profile | That the content describes reality |
| Declarant identity | A credential maps to an identified principal | That the principal is authorized for every treaty action |
| Protocol authority | Credential/mandate is valid for a declared scope and time under an agreed rule | That an institution's legal interpretation is correct |
| Custody | The recorded evidence path is complete and ordered according to the agreed digital protocol | That the initial physical observation was correct or complete |
| Inspection status | A competent inspector's externally issued attestation was received and validated | The software itself cannot perform or substitute for physical inspection |
| Dispute status | A discrepancy is open, resolved, withdrawn or referred, with an audit trail | That an unresolved dispute can be silently treated as resolved |
| Treaty/legal determination | An authorized institution's signed decision is recorded and linked to its governing protocol | A software-generated compliance badge or unilateral calculation |

A signature proves a cryptographic relationship under a trust profile; it is not a truth oracle. Custody completeness improves evidence integrity; it does not prove that the underlying physical process occurred as declared.

## 3. Conservative claim lifecycle

The benchmark uses a monotonic, append-only event history. A current display state is derived from events but never replaces them.

Permitted synthetic states:

1. `declared`: a principal has submitted a generic claim.
2. `evidence_received`: evidence references have been received and bound to the claim.
3. `custody_complete`: all required digital custody links for the declared protocol are present and valid.
4. `inspection_pending`: an authorized institution has recorded that physical verification is pending.
5. `inspection_attested`: an externally issued, valid inspector attestation has been received and checked against the configured authority profile.
6. `disputed`: an authorized participant has opened a discrepancy; the dispute remains visible until a competent resolution event is recorded.
7. `inspection_rejected`: an authorized inspector/institution has issued a negative or inconclusive inspection result under the agreed protocol.
8. `treaty_determined`: an authorized treaty institution has issued a signed legal determination that is imported as an external event.

The lifecycle is not a simple linear ladder. Disputes may be opened from any non-final state and take precedence in the public status view. A later resolution appends a resolution event; it never deletes the dispute. `inspection_attested` does not automatically imply `treaty_determined`. `custody_complete` cannot itself advance to either inspection-attested or treaty-determined.

### Prohibited transitions

The evaluator must reject:
- `declared -> inspection_attested` based only on a valid claimant signature;
- `evidence_received -> treaty_determined` based only on a file digest or signed receipt;
- `custody_complete -> treaty_determined` without an external treaty-institution decision;
- `disputed -> treaty_determined` while the dispute has not been resolved through the competent process;
- any state promotion by a principal whose authority is expired, revoked, or outside scope;
- any deletion or overwrite of evidence, custody, or dispute events to manufacture a clean history.

An institution's authority is represented as an external, versioned mandate reference. The benchmark only checks that the referenced authority assertion is present, valid within the fixture's abstract rules, correctly scoped and not revoked. It does not create real-world authority.

## 4. Evidence-custody record contract

Each synthetic event has:
- stable claim ID and event ID;
- event type and protocol version;
- actor/member pseudonym;
- authority assertion reference where the event requires authority;
- event time and record-receipt time;
- prior-event digest or explicit genesis marker;
- payload digest and digest algorithm;
- evidence status;
- purpose and disclosure class;
- optional supersedes/references relationship.

The fixture contains no raw evidence payloads. All subject matter is fictional and generic. The benchmark validates metadata relationships, not evidence content.

## 5. Required adversarial fixture families

- Signed declaration with no inspection attestation.
- Valid custody but no physical-verification authority.
- Missing, duplicated, stale and contradictory generic evidence references.
- Broken previous-event digest / reordered custody.
- Replayed event identifier.
- Revoked, expired or out-of-scope authority assertion.
- Protocol version mismatch.
- Unauthorized claimant attempts to issue an inspector attestation.
- Unauthorized software transition to treaty determination.
- Unresolved dispute silently cleared or omitted from a summary.
- Protected evidence details disclosed to an unauthorized audience.
- Partial participation or member withdrawal with prior custody history retained.
- Valid external inspection attestation but no treaty determination.
- Valid treaty decision referencing a different claim/protocol digest.

## 6. Hard acceptance gates

- 100% traceability of accepted digital evidence references to the declared claim and protocol version.
- Zero replayed event acceptance.
- Zero custody-chain break accepted as complete.
- Zero revoked/out-of-scope authority event accepted.
- Zero software-only transition into `inspection_attested` or `treaty_determined`.
- Zero unresolved-dispute suppression or deletion.
- Zero unauthorized disclosure.
- Zero event-history overwrite; correction is append-only.
- 100% fixture reproduction by an independent evaluator.

These are structural integrity gates, not evidence that a real-world disarmament regime is verified.

## 7. Metrics

Report each separately, with explicit denominators:
- false acceptance and false rejection;
- missing/duplicate/stale/contradictory evidence detection;
- custody-link completeness;
- replay and duplicate-event rejection;
- authority-scope and revocation rejection;
- illegal-state-transition count;
- unresolved-dispute visibility;
- protocol version mismatch detection;
- privacy exposure and unauthorized disclosures;
- synthetic dispute-resolution latency;
- audit completeness and independent reproduction;
- handling under member withdrawal and partial participation.

Do not create a weighted "verification confidence" number that can make a critical custody or authority failure look acceptable.

## 8. Comparison protocol

Compare only implementations that receive exactly the same synthetic event histories and authority profiles:
1. append-only event ledger with explicit authority checks;
2. naive mutable status document (negative baseline expected to fail overwrite/dispute controls);
3. provenance-only ledger with no external authority model (expected to fail authority-separation controls);
4. candidate GCPN/Mycelix evidence interface.

The baselines are deliberately diagnostic. They are not named treaty systems and must not be represented as an empirical ranking of inspectors or institutions.

A confirmatory evaluation must freeze fixture IDs and seeds, include hidden combinations, reproduce results with a separate evaluator, and report both false acceptance and false rejection. A benchmark should prefer explicit "inconclusive" over false certification when evidence is incomplete.

## 9. Relationship to GCPN and Mycelix

GCPN may transport agreed incident notifications and evidence references, but it is not an inspection authority. Mycelix may provide identity, credential, provenance, governance, dispute and audit primitives, but identity/provenance do not make a claim true. Praxis/NRES may evaluate uncertainty, evidence completeness and decision trade-offs, but do not issue treaty determinations.

## 10. Non-claims

This benchmark does not verify physical dismantlement, model a nuclear facility, inspect material, establish treaty compliance, reduce a country's arsenal, provide a disarmament guarantee, or substitute for diplomacy, inspectors, safeguards, law, or treaty institutions. It is a synthetic test specification for digital evidence integrity and authority boundaries only.
