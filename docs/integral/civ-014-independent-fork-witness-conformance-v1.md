# CIV-014 Independent Fork-Witness Conformance v1

Status: experimental cross-language reference profile, not production witness qualification.

Profile ID: civ-014-independent-fork-witness-v1  
Tracking issue: https://github.com/Luminous-Dynamics/symthaea/issues/7293  
Producer PR: https://github.com/Luminous-Dynamics/symthaea/pull/7298  
Pinned producer commit: 5e2a06f2b6a0aa12c76d652bc249271050a8c0cd  
Pinned producer module blob: eadb5e4209c45d49fe19f7aa488af2ce591218db  
Independent checker: scripts/integral/verify_civ014_independent_fork_witness_v1.py

## Claim ceiling

This profile verifies canonical event/receipt encoding and abstract rejection outcomes across a separate Python implementation. It does not import or run the Rust crate, provide a remote witness implementation, or qualify durable local pending state, network ambiguity recovery, a production trust domain, power-loss behavior, or recovery from real full-local erasure.

## Canonical event

The event domain is the byte string mycelix-civ014-fork-event-v1 followed by a NUL byte. All integers are unsigned big-endian; text uses UTF-8 with an eight-byte big-endian byte-length prefix. Digests are raw 32-byte values, lowercase hexadecimal only in JSON.

The canonical event bytes are, in order:

1. Domain separator.
2. Schema version as u16; currently 1.
3. Length-prefixed log ID.
4. Witness epoch as u64.
5. Previous event count as u64.
6. Previous event tail digest.
7. Accepted generation observed at detection time as u64.
8. Accepted-head digest observed at detection time.
9. Fork generation as u64.
10. First record digest and conflicting record digest.

The event digest is SHA-256 of those bytes. The previous event count and digest are included in the event identity. A frontier with count zero must use the all-zero tail digest. An unknown log/epoch is unavailable; it is not implicit genesis.

Validation rejects an empty log ID, unsupported schema, generation-zero fork, nonzero digest for the generation-zero accepted head, fork generation more than one ahead of the accepted head, identical competing record digests, and an empty prior frontier with a nonzero tail.

## Canonical receipt

The receipt domain is the byte string mycelix-civ014-fork-receipt-v1 followed by a NUL byte. Receipt bytes are the domain separator, length-prefixed log ID, witness epoch, previous event count, previous tail digest, event digest, resulting event count, and resulting tail digest. Integers are unsigned big-endian and counts are u64.

A valid receipt must bind the exact event and previous frontier, and the resulting frontier must preserve log/epoch, increment the count exactly once, and set its tail to that event digest. The receipt digest is SHA-256 of the receipt bytes.

## Frozen positive digests

The independently computed golden fixture uses log ID log-a, witness epoch 7, an empty prior frontier, accepted generation 4 with a digest consisting of byte 0x04 repeated 32 times, fork generation 4, a first record digest of 0x05 repeated 32 times, and a conflicting digest of 0x06 repeated 32 times.

- Event digest: 85a478bb7489447b24b8874b8e0bcdb4ae143cd856174695e155ae4114c16dff
- Receipt digest: 06df505452cc1ef6c45c717084da3f15c89f34958cee5ff0fdd4441bfb03b926

The event and receipt digests were produced by a separate SHA-256/canonical encoder and the implementation was sanity-checked against the standard empty-input and abc SHA-256 vectors. The independent Python checker recomputes both hashes from fixture fields; it does not trust the expected digest fields alone.

## Vector coverage

| Vector IDs | Contract |
|---|---|
| DF001–DF002 | Canonical event and receipt golden digests |
| DF003–DF006 | Reject generation zero, malformed genesis digest, too-far-ahead generation, and identical competing digests |
| DF007–DF008 | Reject malformed empty frontier and event/frontier mismatch |
| DF009–DF011 | Reject skipped receipt frontier, tampered event digest, and tampered receipt digest |
| DF012 | An unprovisioned log/epoch is unavailable, not implicit genesis |
| DF013–DF014 | Exact replay returns its existing receipt; payload tampering fails closed |
| DF015 | Stale frontier cannot append a different event and leaves frontier unchanged |
| DF016 | Cross-log frontier binding is rejected |

## Implementation boundary

This profile qualifies the event/frontier/receipt interface and its reference vectors only. The producer PR defines a test-only in-memory witness. The SQLite adapter is not yet wired to a durable local pending/anchored fork-event journal, and no production remote witness retains these events. The required next steps remain: persist local pending state, append to a separate durable witness, verify/read back the exact receipt, finalize local state, and test crash/timeout/restart at every boundary. A tail hash alone cannot reconstruct an erased event payload.
