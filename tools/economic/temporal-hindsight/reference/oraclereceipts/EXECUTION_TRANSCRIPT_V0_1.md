# Oracle / IVC execution transcript V0.1

This reference composes structurally verified oracle-receipt chains with immutable computation steps and claimed incremental-verifiable-computation (IVC) proof updates.

It is a data and replay contract, not an IVC implementation or proof verifier.

## Step model

Every step binds:
- source/snapshot/registry epoch/query scope through the exact oracle-receipt scope digest;
- a unique step ID, program digest, evaluator-version digest, policy-version digest and query-plan digest;
- exact input-state digest, output-state digest and output digest;
- exact receipt-chain prefix sequence and digest;
- a canonical set of consumed oracle receipt digests;
- optional earlier step IDs superseded by a correction/compensation/revocation;
- IVC protocol/version, parameters digest, verification-key digest, prior/next proof digests, proof artifact digest and a recomputed public-input digest.

The public-input digest is recomputed from the exact step fields, the canonical consumed-receipt set and supersession set. The transition digest additionally binds the IVC envelope.

## Commitment-carrying oracle advice

The IVC envelope also carries a three-part advice commitment:
- PriorAdviceCommitmentDigest: the cumulative advice commitment at the current state/proof head;
- AdviceDeltaCommitmentDigest: a domain-separated commitment to this step's exact receipt-prefix digest, sequence and canonical consumed receipt-digest set;
- NextAdviceCommitmentDigest: a domain-separated hash-chain extension of the prior advice commitment with the new advice delta.

The replay verifier recomputes the delta and next commitment, and requires the prior commitment to equal the current advice head. The resulting advice commitment is reconstructed alongside state, proof and receipt-frontier heads and is included in the replay-result integrity seal. A correction therefore appends a new advice delta and a new cumulative advice commitment; it cannot silently rewrite the previously committed oracle inputs.

This adapts the *commitment-carrying IVC* idea from Nebula: its proof carries an incremental commitment to non-deterministic advice across steps, supporting efficient memory/read-write arguments. Here, the "advice" is the canonical set of externally supplied oracle receipts. The present SHA-256 chain is a structural commitment convention, not a hiding vector commitment or an IVC proof. It only identifies the exact receipt inputs; an eventual IVC circuit/proof must demonstrate that its computation consumed those inputs, and an independent verifier must still verify the proof and the oracle receipt signatures/authority. See Arun & Setty, *Nebula: Proving Machine Executions via Folding Schemes* (IEEE S&P 2026): https://www.microsoft.com/en-us/research/publication/nebula-proving-machine-executions-via-folding-schemes/.

## Replay invariants

1. Execution events are contiguous, append-only and previous-digest chained.
2. A step consumes the current state head and current IVC proof head; a stale state/proof head fails closed.
3. A step's receipt frontier must exactly match a real prefix of the structurally verified oracle-receipt chain.
4. A step cannot consume a receipt beyond its declared prefix.
5. Receipt frontier sequence is monotone across the state/proof history. A later step cannot rewind to an earlier oracle prefix.
6. Correction, compensation and revocation steps supersede earlier applied steps and must consume a strictly later receipt frontier than the steps they supersede.
7. An OutcomeUnknown event reserves the attempted step ID but does not mutate state, proof or frontier. No later mutation is accepted until the exact event is reconciled.
8. ReconciledApplied applies the prepared step only after its preconditions are rechecked; ReconciledNotApplied leaves the state and proof heads unchanged. The reserved step ID is not reusable.
9. Prior event, step and receipt history is never overwritten. A correction appends a new event and a new state/proof head.

## Trust boundary

Replay validates structure only. It does not verify:
- oracle signatures or key-epoch/revocation state;
- authority grants, roles or jurisdiction;
- source-universe completeness or signed checkpoint/witness trust;
- IVC proof validity, soundness, setup, verifier key or public-input semantics;
- correctness of the computation, policy legitimacy, or oracle answer truth.

The receipt-chain structural verifier still reports oracle signature, key, authority, completeness and truth as NotEvaluated. The execution transcript reports IVC proof verification as NotEvaluated. Updating an observed proof-head digest is not a claim that the proof has been verified.

## Correction semantics

A correction is represented by:
1. a new immutable oracle receipt that supersedes an earlier receipt;
2. a new RecomputeAfterCorrection step that refers to an earlier applied step and consumes the later receipt prefix;
3. a new output state digest and next IVC proof digest.

The original receipt and step remain in the transcript. There is no last-write-wins mutation.

## Model checking

formal/OracleIVCExecution.tla checks:
- state, proof and cumulative advice-commitment heads advance together;
- receipt-frontier high-water mark never regresses;
- unknown outcomes freeze state/proof/frontier heads;
- the prepared target is exactly the next state/proof/advice/frontier.

Three independent mutants must each be caught:
- OracleIVCExecution-Regress-Mutant.cfg: rewind the receipt frontier;
- OracleIVCExecution-Proof-Mutant.cfg: advance only the proof head;
- OracleIVCExecution-Pending-Mutant.cfg: mutate while an unknown result is unresolved.

The finite model has MaxSteps = 2. It is bounded model checking, not an unbounded proof of the Go implementation.

## Benchmarks

The benchmark measures structural transcript replay at 1/10/100/1,000/10,000 steps using a fixed, structurally valid one-receipt chain. It does not represent full end-to-end IVC performance.

A proper comparison must separately measure:
- full computation replay + fresh proof;
- result-only incremental view maintenance;
- incremental computation + fresh proof;
- incremental computation + IVC proof update;
- independent verification of the resulting proof and authenticated source/checkpoint inputs.

Report at least 30 warmed independent trials, raw samples, p50/p95/p99, hardware/toolchain, proof size, public-input size, verification key/parameters, state/transcript size, correction amplification, and time to a fully qualified result. Never compare a result-only IVM delta to a verified proof update as if they provide the same guarantee.

## Run

    go test ./...
    go test -race ./...
    go vet ./...
    go test -run '^$' -bench . -benchmem

The hosted workflow also runs 30-repeat adversarial tests, bounded TLC, each of the three required mutant counterexamples, a one-iteration benchmark smoke and a SHA-256 evidence bundle.
