# Challenge-v7 multi-agent conductor gate

This is the required real-conductor follow-up to the executable protocol model.
It is intentionally not marked complete by this patch series.

## Topology

Run at least three independent conductors with separate databases and networking:

- **V** — verifier and challenge author;
- **P1** — intended prover; and
- **P2** — competing prover using the same challenge material.

A fourth read-only observer is recommended for state convergence assertions.
Use explicit network partitions rather than multiple cells in one conductor.

## Required scenarios

1. **Pre-settlement refusal**
   - V issues a short-lived challenge.
   - P1 submits one validly framed artifact before expiry.
   - V attempts request construction, proof receipt, anchor receipt, and policy
     acceptance before `settlement_not_before`.
   - Every operation must fail with the settlement error.

2. **Settled single consumer**
   - Heal all links and wait until every conductor sees exactly one
     `ChallengeToProof` link.
   - `build_scoped_verifier_request` must return byte-for-byte JSON matching the
     native verifier schema and the verifier-emitted request hash.
   - A substituted request hash or result hash must be rejected.
   - Exact valid proof and anchor receipts may progress to policy acceptance.

3. **Concurrent consumers before convergence**
   - Partition P1 and P2 from one another while both can author.
   - Both submit against the same unexpired challenge.
   - Heal the partition and wait until V and the observer see both links.
   - Request construction, both receipts, and policy acceptance must fail.
   - `get_effective_proof_state` must return `ChallengeConsumptionConflict` for
     both artifacts.

4. **Late competitor after apparent acceptance**
   - Hide P2's pre-expiry artifact from V through settlement.
   - Let V create receipts and an acceptance for P1 after seeing one consumer.
   - Reveal P2 and wait for convergence.
   - The effective state must change to `ChallengeConsumptionConflict`; no API
     may continue returning an effective accepted state merely because the
     immutable acceptance record exists.

5. **Index poisoning and deletion attempts**
   - Attempt wrong-base, wrong-target, wrong-author, duplicate, and deleted
     challenge/proof/receipt/policy links.
   - Invalid links and all protocol-link deletions must fail integrity
     validation. A duplicate valid link must at minimum force fail-closed
     conflict rather than silently deduplicate into acceptance.

6. **Verifier transcript substitution**
   - Mutate each request field independently: relation, implementation,
     circuit digest, parameter identity, source manifest, scope, either
     pseudonym, context hash, nonce hash, and proof bytes.
   - Mutate validity, proof hash, evidence hash, and error code in the result.
   - Every mutation must reject receipt creation.

## Evidence bundle

The CI lane should retain conductor logs, authored action hashes, network state
snapshots, exact native-verifier request/result JSON, binary and build hashes,
and a machine-readable scenario report. The lane passes only after every
conductor converges to the expected fail-closed state.
