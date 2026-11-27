# Oracle receipts for temporal evidence V0.1

This module preserves signed source, expert and human judgments as immutable query-bound evidence inputs. It is designed for later incremental-verification composition; it does not attempt to prove that an oracle's answer is true.

## Epistemic dispositions

The answer disposition is explicit and typed:

- Affirm
- Reject
- Abstain
- UnableToAssess
- NoResponse

NoResponse is not an empty affirmative, and an explicit abstention is not a missing response. Non-response carries an empty answer payload; the other dispositions require a payload explaining the answer or limitation.

## What each receipt binds

A receipt commits to:
- source/snapshot/registry identity and registry epoch;
- query-profile, exact query digest, input commitment and verifier-selected evidence frontier;
- oracle identity, key epoch, authority-grant reference and authority-scope digest;
- answer schema, disposition, canonical payload digest and evidence references;
- recorded-at time and a separate valid-time interval;
- optional superseded receipt reference;
- signature scheme, signature bytes and exact signed-message digest.

Evidence references are sorted canonically for commitment generation. A receipt's recorded time is independent of its valid-time interval, so a later correction can make a backdated assertion without rewriting its predecessor. Superseding receipts must point to an earlier receipt in the chain. Conflicting answers from different oracles can coexist; the chain does not impose last-write-wins.

## Submission bounds and failure receipts

The chain verifier preflights receipt count, identifier sizes, answer payload sizes, signature sizes, evidence-reference count and aggregate encoded bytes before hashing the full submission. Failure certificates bind the expected tail digest and a domain-separated digest of the submitted receipt records. Oversized submissions receive a bounded length/prefix/suffix commitment so producing the failure receipt does not copy or hash arbitrarily large payloads. Valid-time intervals support pre-epoch timestamps; recorded-at remains a separate positive system timestamp.

## Verification boundary

`VerifyOracleReceiptChainV1` checks structural properties:
- receipt IDs and sequence positions are unique/contiguous;
- previous-receipt digest chain and expected count/tail anchor match;
- each receipt is bound to the exact query, input commitment and evidence frontier;
- answer payload, evidence reference, valid-time and signed-message digests are consistent;
- supersession references point to earlier receipts.

It does **not** cryptographically verify the signature, key epoch status, revocation, authority grant, role/jurisdiction scope, source-universe completeness, evidence truth or the judgment itself. Every corresponding result field stays `NotEvaluated`.

A valid signature would mean only that a key signed a particular answer to a particular query/input. It would not establish expertise, authority, source completeness or truth. The next integration step is to reuse the repository's existing DID/key lifecycle and RFC 9162 checkpoint contracts rather than introducing a new trust system.

## Relation to incremental verifiable computation

Goldwasser and Kalai, *Incrementally Verifiable Oracle Aided Computation* (2026), studies verifiable computation where answers from external oracles/expertise are part of the computation. This reference module models the data contract for those inputs. An eventual IVC circuit or proof must demonstrate that the computation consumed these exact authenticated receipts; it must not turn the fact that an oracle answered into proof that the answer was correct.

Paper: https://arxiv.org/abs/2611.14698

## Run

    go test ./...
    go test -race ./...
    go vet ./...
    go test -run '^$' -bench . -benchmem

The benchmark measures in-memory canonical receipt verification only. It excludes cryptographic signature verification, key/authority lookup, transparency consistency proofs, source-roster completeness, network latency and an IVC prover.
