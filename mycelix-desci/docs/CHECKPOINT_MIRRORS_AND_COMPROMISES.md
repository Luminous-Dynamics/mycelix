# Checkpoint Mirrors and Witness-Compromise Intervals

## Signed mirror observations

A governed transparency witness may publish a checkpoint at an independently
operated HTTPS location and sign a mirror observation containing:

- checkpoint hash;
- witness actor and organization;
- immutable, hash-addressed mirror URI;
- witness-observed time;
- signing public key; and
- Ed25519 signature.

The API accepts observations only when PostgreSQL is authoritative. The
observation and its publication-outbox message commit in one transaction. The
mirror URI must contain the canonical checkpoint hash in its path, may not use
query parameters, and the database records a separate server acceptance time; a
mutable `latest` URL is rejected.

An external verifier should fetch the mirror URI, parse the published checkpoint,
recompute its canonical checkpoint hash, compare it with the signed observation,
and then verify the witness against the governance history at `observed_at`.
The API does not claim that recording an observation proves continued HTTP
availability.

## Compromise intervals

Witness compromise is represented by one or more governed intervals:

```text
[compromised_from, restored_at)
```

An open interval has no `restored_at`. Intervals cannot overlap. Recording a
compromise is a critical threshold-governance action.

A later forensic notice recalculates checkpoint and mirror quorum by signature
time:

- signatures before an affected interval remain valid;
- signatures inside the interval stop contributing;
- signatures after a governed restoration may contribute again if the witness
  key and authorization are otherwise active.

No signature or observation is deleted. Historical evidence remains available
with its current validity classification.

## Readiness

`DESCI_MIN_CHECKPOINT_MIRROR_ORGANIZATIONS` sets the minimum number of distinct
organizations with currently valid signed observations for the latest governed
checkpoint. Readiness re-verifies each stored observation against the current
governance projection, including retroactive compromise intervals; it does not
trust a raw database row count.

Mirror and witness organization thresholds are separate. A deployment should
require both, and should operate mirrors under administrative and infrastructure
independence rather than merely different account names.
