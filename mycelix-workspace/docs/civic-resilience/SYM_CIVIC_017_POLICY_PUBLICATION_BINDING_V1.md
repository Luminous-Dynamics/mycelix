# SYM-CIVIC-017 — policy-publication binding preflight v1

Status: design/research preflight only; **not qualified**

Parent subject: `0a0b5bbddbac1e611ee9664896c18387a14373f5`

## Boundary

017 closes the publication gap immediately above 016:

`effective Witness Set + effective Witness Quorum -> signed Policy Parameters publication bound to the exact dependency snapshot`

ARP-04 (September 2026) requires the reconciliation server to publish a Policy Parameters Document at `/.well-known/arp-policy-parameters` as a COSE_Sign1. The payload carries the effective Witness Set and effective Witness Quorum, with the Witness Set deterministically sorted; the document also carries a Publication Timestamp and the freshness/notarisation parameters. citeturn362648search0turn824597view1

016 proved the synthetic effective-set relation. 017 asks the stronger local question: can a verifier detect when the published snapshot no longer describes the source agreements or policy inputs that generated it?

## Model

The synthetic Policy Parameters publication contains a six-element `wire_payload` shaped after ARP-04 §7.4.1:

1. Publication Timestamp;
2. permitted signature algorithms;
3. per-predicate policy entries;
4. Pattern-Library / Policy-Version transitions;
5. the effective Witness Set and effective Witness Quorum;
6. response freshness tolerance and ledger-head notarisation interval.

The qualification envelope additionally carries the exact source Agreement Hash identifiers and an independent content digest for each source Agreement, plus a sealing-key reference, payload digest, and synthetic signature-binding digest.

The source content digest is explicitly a qualification control. ARP-04 §5.23.3 states that a production Agreement Hash is a SHA-256 digest over the deterministically encoded Agreement contents; this research preflight uses a content-digest surrogate so that source-content replacement cannot silently reuse an old publication without pretending to implement deterministic CBOR or COSE cryptography. citeturn362648search0

## Failure model

017 rejects or marks stale publications when the effective set or quorum is altered, dependencies are omitted, source content changes under an old publication, signer binding drifts, digests no longer match, canonical ordering is changed, freshness/notarisation inputs drift, publication time moves backwards, or an unknown critical extension appears.

A later timestamp with otherwise identical canonical content is permitted as an explicit re-publication event.

## Ceiling

GREEN would establish only synthetic publication/dependency binding. It does **not** establish actual COSE signature validity, key resolution, Transparency Service notarisation, clock synchronization, witness honesty, declared independence, common-control absence, register-content truth, legal authority, or operational safety.

It also does not establish that the deployment has discovered every Agreement it holds; the fixture's source Agreement set is authoritative only within this research model.

## Design rule

`Publish the effective witness state canonically with dependency identifiers and content digests; make drift produce a new publication identity rather than silently mutating an old snapshot.`

References:

- draft-hillier-scitt-arp-04, §5.23.2.5, §5.23.3, §7.4.1: https://datatracker.ietf.org/doc/draft-hillier-scitt-arp/
- RFC 3986: https://www.rfc-editor.org/rfc/rfc3986
