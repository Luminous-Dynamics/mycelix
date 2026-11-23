# PCOE SCITT Receipt Continuity Addendum v0.1

**Status:** experimental interoperability and adversarial-qualification profile; not an IETF conformance claim, a transparency-service certification, or proof of physical truth.  
**Snapshot date:** 23 November 2026.  
**Parent implementation:** PCOE reference-kernel PR #6526, tested head \`4cd588c7bd4081e169160ba37328fc5c91f49afe\`.

## 1. Why this increment exists

The existing PCOE transparency profile correctly separates a signed statement from its registration receipt and from the physical, contractual, and financial facts that the statement may describe. The new SCITT receipt work exposes a further boundary that must be explicit:

> **Inclusion is not continuity; continuity is not completeness; and neither is factual truth.**

A receipt may prove that a statement is represented in a transparency service's verifiable data structure. That alone does not prove that the service's view is consistent with the checkpoint a verifier previously trusted, that the issuer registered every expected event, or that the statement is true.

PCOE therefore treats transparency as a time-qualified evidence layer over an obligation graph, not as a new source of economic or physical authority.

## 2. Standards snapshot and stability policy

| Artifact | Snapshot status on 23 November 2026 | PCOE treatment |
|---|---|---|
| [RFC 9943 — SCITT architecture](https://www.rfc-editor.org/rfc/rfc9943) | June 2026, Standards Track Proposed Standard | Stable architectural foundation for signed-statement transparency |
| [RFC 9942 — COSE Receipts](https://www.rfc-editor.org/rfc/rfc9942) | June 2026, Standards Track Proposed Standard | Stable receipt framework and VDS / proof-type extensibility |
| [draft-ietf-scitt-scrapi-11](https://datatracker.ietf.org/doc/draft-ietf-scitt-scrapi/) | Active Internet-Draft; datatracker shows the RFC Editor queue / first-edit process as of the November snapshot | Pin as an API candidate and re-check before implementation release; do not call it a published RFC |
| [draft-ietf-scitt-receipts-mmr-profile-00](https://datatracker.ietf.org/doc/draft-ietf-scitt-receipts-mmr-profile/00/) | New SCITT WG Internet-Draft dated 21 November 2026; WG document, I-D exists | Experimental VDS profile only; do not hard-code provisional registry values |

The MMR draft uses provisional / not-yet-assigned identifiers for its VDS and consistency tree-size header label. Implementations MUST pin an exact future profile revision and registry assignments before interoperating. This addendum intentionally does not encode those provisional numeric values.

The MMR profile is not mandatory for PCOE. A SCITT service may expose a different RFC 9942-compatible VDS profile. The verifier must dispatch only to an explicitly supported, pinned profile and fail closed on an unknown one.

## 3. Registration lifecycle: HTTP acceptance is not registration

The current SCRAPI-11 draft describes asynchronous registration. In that draft's flow:

- \`202 Accepted\` means registration has been accepted but is still running.
- \`204 No Content\` when retrieving the entry means registration is still in progress.
- \`200 OK\` makes a receipt available for verification.
- \`404 Not Found\` can mean there is no receipt, including because asynchronous registration failed.

PCOE maps these to independent dispositions:

| Observation | PCOE transparency state | May satisfy an inclusion gate? |
|---|---|---|
| Request sent / transport acknowledgement | \`Pending\` | No |
| HTTP 202 | \`Pending\` | No |
| HTTP 204 on receipt lookup | \`Pending\` | No |
| HTTP 200 with receipt bytes | \`ReceiptAvailable\` until cryptographic verification completes | No |
| Receipt verifies against the exact statement, configured service, supported VDS and required checkpoint policy | \`RegisteredAndVerified\` | Yes, for statement inclusion only |
| Explicit registration failure / terminal 404 | \`Rejected\` or \`Indeterminate\` according to the service response and policy | No |
| Timeout, malformed response, unavailable service or ambiguous status | \`Indeterminate\` | No |

Do not map HTTP success classes directly to \`PaidFinal\`, \`VerifiedProduction\`, \`Accepted\`, or any other PCOE economic state. A verified transparency receipt is only one predicate in the larger obligation graph.

## 4. MMR receipt semantics that PCOE must preserve

This section tracks the 21 November 2026 MMR profile draft. It is a test-design input, not a frozen wire-format specification.

### 4.1 Verify the detached payload against the proof

The draft requires the receipt payload to be detached. The verifier must recompute the relevant root from the supplied value and inclusion path, then verify that it matches the signed receipt payload. Verifying a COSE signature without recomputing the proof-derived root is insufficient.

### 4.2 Inclusion does not establish checkpoint continuity by itself

The draft's inclusion receipt carries no tree size. The inclusion proof establishes that a value is a node in the tree; it does not, by itself, bind that inclusion to the historical checkpoint a verifier has already trusted.

PCOE must retain a trusted starting point:
- supported VDS profile identifier;
- trusted tree size;
- trusted accumulator / root state;
- exact transparency-service identity and trust-policy revision;
- the receipt and any required proof material.

A new checkpoint becomes trusted only after its signature and required consistency proof are verified from the previously trusted checkpoint. A changed or newer root is not trusted merely because the service returned it.

### 4.3 Consistency is a separate proof obligation

A consistency proof must connect the stored prior checkpoint to the new checkpoint under the same supported VDS profile. PCOE must reject or hold when the proof is missing, malformed, uses an unexpected profile, skips a required trusted checkpoint, or does not terminate at the declared newer tree size / accumulator.

The MMR draft defines cumulative consistency proofs. A verifier must retain its trusted starting VDS, tree size, and accumulator rather than accept a service-supplied "latest checkpoint" as a new trust anchor.

### 4.4 Bind the application leaf, not just a tree node

The MMR draft notes that whether a node is a leaf and what its entry commits to depend on the application's leaf-commitment scheme. Its security considerations also note that an empty inclusion path does not bind an index.

Therefore, the PCOE application statement / leaf-commitment profile must bind the exact statement digest and, where order or completeness matters, the source sequence / leaf position. An index supplied outside the authenticated commitment is not an integrity guarantee.

### 4.5 Canonical protected headers

The draft requires deterministic CBOR encoding for the protected header and rejection of duplicate labels and non-deterministic encodings. PCOE must not normalize malformed signed bytes and then treat the normalized representation as what was signed.

Unknown or unsupported critical parameters, invalid signature algorithms, unsupported VDS identifiers, and missing required profile parameters must never fall through to a permissive default.

## 5. PCOE receipt verification pipeline

Use discrete stages with durable outputs. Do not collapse the stages into a single boolean.

1. **Statement admission:** preserve the exact signed-statement bytes, issuer/key reference, application-profile version, subject / obligation ID, payload digest, evidence digests, event time, record time, and correction / supersession references.
2. **Issuer authentication:** verify the COSE signed statement, protected headers, issuer key binding, key validity / revocation at the relevant time, and the exact payload digest.
3. **Registration resolution:** follow the transparency service's entry identifier; distinguish pending, rejected, indeterminate, and receipt-available states.
4. **Receipt verification:** verify the COSE receipt signature and exact service identity; decode only a pinned supported VDS profile; recompute inclusion from the detached value and proof.
5. **Continuity verification:** verify consistency from the verifier's previously trusted checkpoint; persist the new checkpoint only after successful verification.
6. **Completeness reconciliation:** compare registered statements against an independently defined expected-event set, source sequence, meter interval manifest, contract milestone schedule, or rail statement. A missing receipt is not proof that an event did not occur.
7. **Truth-scope evaluation:** independently evaluate metrology, verifier accreditation, authority, contract acceptance, execution and settlement. Transparency does not satisfy any of those predicates.

### Required verifier output

The independent verifier should emit a deterministic report containing at least:

- exact input statement and receipt digests;
- issuer identity, key ID and key-validity decision;
- transparency-service ID and policy revision;
- receipt media type and exact VDS profile revision;
- inclusion proof result and bound leaf / statement identifier;
- trusted prior checkpoint reference and newly verified checkpoint reference;
- consistency proof result and tree-size transition;
- freshness / rollback / equivocation disposition;
- expected-set or source-sequence reconciliation result;
- unsupported / missing evidence and explicit limitations.

A report generated by the issuer's application is not a substitute for independently verifying the bundle.

## 6. Temporal completeness and anti-omission

An append-only log can make registered statements tamper-evident without proving that every expected statement was registered.

PCOE requires an expected-set source independent of the party whose omission is being tested. Examples include:
- signed meter sequence ranges and interval manifests;
- a contract's scheduled milestone IDs;
- bank / payment-rail statement references;
- independently issued custody inventory events;
- an agreed registry export / reconciliation window.

The reconciliation result must distinguish:
- observed and registered;
- observed but not registered;
- registered but not present in the independently expected set;
- expected but absent from the source;
- source unavailable or sequence incomplete;
- explicitly superseded / corrected.

Do not infer "no event", "no production", "no payment", or "no incident" from the absence of a transparency receipt.

## 7. Independence and trust-root policy

Multiple URLs, log shards, or signing keys do not automatically represent independent transparency services. Qualify:
- legal operator and beneficial control;
- cloud / hosting and administrative domain;
- signing-key custody and rotation authority;
- software supply chain and deployment pipeline;
- witness / monitor dependencies;
- checkpoint distribution and independent gossip paths;
- contractual and commercial conflicts.

A verifier must pin the service identity and key policy independently from the statement issuer. Key rotation, compromise, revocation, or an unresolved checkpoint fork moves dependent obligations to review / indeterminate until reconciled.

## 8. Privacy and evidence minimization

Transparency is public or broadly auditable by design; PCOE must not publish personal information, account identifiers, exact household consumption, confidential contract terms, or low-entropy secrets merely because a digest is available.

A plain hash of a guessable value can be brute-forced. Use an approved commitment / disclosure profile with adequate entropy and access policy; disclose only the fields needed by a given verifier. Preserve restricted evidence by protected reference and rights metadata where public distribution is not lawful or safe.

## 9. Industry comparison and intended differentiation

| System / standard | Existing contribution | PCOE objective |
|---|---|---|
| SCITT RFC 9943 + COSE Receipts RFC 9942 | Interoperable signed-statement transparency and verifiable data-structure receipts | Bind registered statements into economic obligations without confusing inclusion with truth |
| SCITT SCRAPI | Reference HTTP registration / receipt lifecycle | Preserve pending vs registered states and test exact asynchronous behavior |
| Certificate Transparency RFC 9162 | Signed tree heads and Merkle inclusion / consistency proofs | Reuse the mature continuity concept; apply it to obligation statements and additional VDS profiles |
| Sigstore Rekor | Software supply-chain transparency, inclusion / consistency proofs, checkpoints and signed entry timestamps | Learn from offline verification, checkpoint persistence and monitoring; avoid creating a competing general-purpose log |
| PCOE | Cross-domain obligation qualification | Add expected-set reconciliation, physical truth-scope, authority, acceptance, external effect and settlement invariants |

The goal is not to claim that PCOE's log is faster or more secure than Rekor or a specialist transparency service. It is to make evidence from those services composable with the rest of the obligation lifecycle and independently reproducible.

## 10. Qualification ladder and benchmark targets

All targets below are proposed qualification goals, not achieved results or standards-mandated thresholds.

| Level | Property | Target |
|---|---|---|
| L0 — parsing | Deterministic CBOR / schema / profile parsing | 100% of supported-profile vectors parsed identically by two implementations |
| L1 — statement | Issuer, key, protected-header and payload binding | 0 malformed or mismatched signed statements accepted |
| L2 — inclusion | Detached payload and VDS inclusion proof | 0 bad or mismatched inclusion proofs accepted |
| L3 — continuity | Trusted checkpoint and consistency proof | 0 rollback, fork, skipped-anchor or invalid-consistency cases accepted |
| L4 — completeness | Expected-set / source-sequence reconciliation | 100% of deliberately omitted expected events surfaced as exceptions |
| L5 — truth boundary | Registered false claim is not physical verification | 0 transparently registered claims promoted to verified production without independent physical evidence |
| L6 — reproduction | Independent verifier rebuilds the disposition | 100% exact agreement over the published deterministic vector corpus |

Operational reporting should include p50 / p95 / p99 registration latency, receipt-resolution latency, proof-verification latency, proof / bundle bytes, checkpoint age, consistency-chain length, monitor disagreement detection time, and manual reconciliation hours. Freshness limits must be set by the relevant risk policy and service contract, not invented as universal protocol constants.

The independent implementation should be written separately from the issuer and should consume only the published bundle plus pinned trust material. Reuse of the same verifier library is not independent reproduction.

## 11. Adversarial corpus

The companion file \`pcoe-scitt-mmr-receipt-scenarios.v0.1.json\` defines 16 research scenarios for:
- asynchronous registration and terminal registration failure;
- detached payload mismatch;
- malformed / non-deterministic protected headers;
- unsupported VDS profiles and provisional identifiers;
- inclusion without checkpoint continuity;
- empty inclusion path without index-bound leaf commitment;
- missing, malformed, discontinuous, rolled-back or forked consistency proofs;
- receipt replay across subjects;
- selective omission and sequence gaps;
- correlated transparency-service operators;
- issuer-key rotation / revocation ambiguity;
- a registered but physically false statement.

These are proposed adversarial scenarios, not executable tests. They do not change the existing 83-scenario count or the 38-test exact-head result for PR #6526.

## 12. Explicit non-claims and implementation gates

This addendum does not:
- implement a SCITT client or transparency service;
- verify COSE signatures, CBOR proofs, or an MMR receipt;
- claim SCRAPI conformance or finality of an asynchronous registration;
- make an Internet-Draft's provisional identifiers stable;
- establish that a statement is complete, physically true, legally authorized, contractually accepted, or paid;
- qualify multiple transparency services as independent;
- resolve Mycelix production P0 issues #6505 or #6516.

Implementation must follow separate gates: pinned RFC / draft revision; vetted CBOR and COSE libraries; explicit key and trust-policy management; deterministic vectors; independent implementation; adapter integration tests; and only then deployment qualification.

## References

- [RFC 9943 — An Architecture for Trustworthy and Transparent Digital Supply Chains](https://www.rfc-editor.org/rfc/rfc9943)
- [RFC 9942 — CBOR Object Signing and Encryption (COSE) Receipts](https://www.rfc-editor.org/rfc/rfc9942)
- [SCITT Reference APIs, draft-ietf-scitt-scrapi-11](https://datatracker.ietf.org/doc/draft-ietf-scitt-scrapi/)
- [COSE Receipts for MMRs, draft-ietf-scitt-receipts-mmr-profile-00](https://datatracker.ietf.org/doc/draft-ietf-scitt-receipts-mmr-profile/00/)
- [RFC 9162 — Certificate Transparency Version 2.0](https://www.rfc-editor.org/rfc/rfc9162)
- [Sigstore Rekor specification](https://github.com/sigstore/architecture-docs/blob/main/rekor-spec.md)
- [Sigstore security model](https://docs.sigstore.dev/about/security/)
