# Integral Interop 1 — Finality Commitment Mutation Matrix

Status: **ReferenceModelOnly**

This matrix records the semantic-binding chain hardened by the D6M/D6N/D6P/D6O/D6S work:

**D6M observation → D6N assessment → D6P witness/composition → D6O current-finality receipt → D6S canonical derivation receipt → D6X qualified closure.**

The purpose is not to claim that a hash establishes authority. A commitment establishes integrity/identity for the committed representation; authoritative reconstruction and cross-object equality establish whether that representation belongs to the qualified semantic chain.

## Boundary matrix

| Layer | What is committed | Authoritative binding requirement | Mutation coverage |
|---|---|---|---|
| D6M | Complete external observation payload | Observation must still match effect, route, lifecycle, frontier, and authoritative observation-set semantics | Every committed observation field is mutated independently |
| D6N | Complete assessment item, including exact D6M observation commitment | Assessment is reconstructed from authoritative D6M evidence and observation-set semantics | Every committed assessment field is mutated independently |
| D6P | Witness/composition identity, including exact D6N assessment-item commitment and D6O receipt identity | Witness join must agree with exact D6N assessment, D6M evidence, D6O receipt, set, lifecycle profile, and frontier | Cross-object substitution and self-recommitted substitution tests |
| D6O | Current-finality receipt, including composition commitment | Receipt must be reconstructed from authoritative D6P composition | Receipt/composition identity and semantic projection checks |
| D6S | Canonical derivation receipt | Projection/environment/profile/D6P current receipt set must reconstruct the receipt | All material D6S receipt fields are mutation-tested |
| D6X | Candidate-bound closure certificate plus candidate-independent closure identity | Closure must reconstruct selected dependency/reference/resolution sets and validate required D6P inputs | Every semantic certificate field is mutation-tested |

## Required negative property

A self-consistent replacement must not become authoritative merely because its commitment was recomputed.

The intended pattern is:

1. mutate a semantic object;
2. recompute its local commitment;
3. preserve or recompute downstream commitments as necessary;
4. submit the result at the authoritative boundary;
5. require rejection unless the authoritative source itself has changed accordingly.

This distinction matters because a cryptographic commitment proves consistency with a preimage, not that the preimage is the authoritative observation or derivation history. This is consistent with provenance models that separately represent entities, derivations, activities, and validity constraints. citeturn0search0turn0search1turn0search12

## Current executable evidence

The reference implementation currently contains:

- D6M semantic mutation coverage and authoritative self-recommitment rejection.
- D6N assessment mutation coverage and authoritative self-recommitment rejection.
- D6P cross-object assessment substitution rejection.
- D6P D6N→D6M self-consistent substitution rejection.
- D6P receipt/composition substitution checks.
- D6M finality receipts now have their own deterministic commitment, require the receipt evidence root to equal the authoritative observation evidence root, and the D6M ledger rejects receipts whose commitment does not match.
- D6S complete material receipt-field mutation coverage.
- D6X complete certificate-field mutation coverage.

## Remaining interoperability work

The next conformance layer should be generated golden vectors for the complete D6M→D6N→D6P→D6O→D6S→D6X chain.

Those vectors should be independently reproduced from the declared serialization/canonicalization contracts before any cross-language interoperability claim is made. D6M now declares `D6M_OBSERVATION_COMMITMENT_SERIALIZATION = serde-json-tuple-v1`; D6N declares `D6N_ASSESSMENT_COMMITMENT_SERIALIZATION = serde-json-struct-v1`. D6M finality receipts now declare `D6M_FINALITY_RECEIPT_COMMITMENT_SERIALIZATION = serde-json-struct-v1`. Both are deterministic Rust/Serde reference-model contracts, not cross-language canonicalization specifications. D6S explicitly defines the D6S-CANON-1 canonical encoding. The distinction should remain explicit rather than treating all six layers as already cross-language canonical. A future wire-level contract can use an established canonical JSON scheme such as RFC 8785, which exists specifically to make JSON representations invariant and hashable across implementations, but adopting it would itself be a protocol change requiring versioned vectors and migration evidence. citeturn0search0turn0search1

Claim ceiling: **ReferenceModelOnly**.
