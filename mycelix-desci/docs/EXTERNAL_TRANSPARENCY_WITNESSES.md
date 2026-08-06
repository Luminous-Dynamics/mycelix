# External Transparency Checkpoint Witnesses

**Status:** experimental external-observation protocol

A locally published Merkle checkpoint proves consistency only to the system that produced it. Mycelix-DeSci now supports independently governed witnesses that sign the exact checkpoint hash and append their attestations to the governance journal.

## Authority lifecycle

Witness enrollment and revocation are threshold-governed actions.

```json
{
  "type": "authorize_transparency_witness",
  "witness": {
    "actor": "did:key:example-witness",
    "organization": "org:independent-observatory",
    "public_key": [0, 1, 2],
    "valid_from": "2026-08-05T00:00:00Z",
    "valid_until": null,
    "revoked_at": null
  }
}
```

The abbreviated public-key array above is illustrative; the real value must contain exactly 32 bytes.

Witness keys must be globally distinct from scientific actor keys, governance administrators, and authority-service keys. A governed witness signs:

- checkpoint hash;
- witness actor;
- witness organization;
- witness time;
- witness public key.

The surrounding governance envelope must use the same actor, public key, and timestamp. The authority acceptance service then signs the server-observed receipt time.

## Publishing and witnessing

1. Generate and publish a checkpoint through the normal administrator path.
2. Deliver the exported checkpoint to an independent witness through an authenticated channel.
3. The witness independently obtains the credential and governance histories, recomputes both Merkle roots, and verifies the checkpoint.
4. The witness appends a signed witness event.

CLI example:

```bash
mycelix-desci credential-governance-witness-checkpoint \
  --registry ./data/scientific-credentials.json \
  --bootstrap-trust-file ./config/credential-bootstrap-trust.json \
  --acceptance-signing-key-file ./config/authority-receipt-signing.key \
  --governance ./data/scientific-credential-governance.json \
  --witness-actor did:key:example-witness \
  --witness-organization org:independent-observatory \
  --witness-signing-key-file ./witness.seed \
  --checkpoint-file ./checkpoint.json
```

A witness actor can attest to a checkpoint only once. Multiple actors from one organization remain visible, but count as one organization for the deployment readiness gate.

## Deployment policy

`DESCI_MIN_CHECKPOINT_WITNESS_ORGANIZATIONS` controls how many distinct organizations must have witnessed the latest published checkpoint before readiness passes.

- `0`: disable the gate; development/cutover only.
- `2` or more: recommended starting point for a real deployment.

The repository `.env.example` and Compose configuration require two organizations. The library default remains zero for backwards-compatible embedding, so production operators must configure the gate explicitly.

## Revocation and compromise intervals

Revocation stops future witness attestations. A critical governed
`record_transparency_witness_compromise` action can additionally mark one or
more historical intervals during which signatures from the witness do not count.
The interval must already be historical when executed; governance cannot
pre-authorize a future restoration.

Previously recorded signatures remain in the journal. Quorum is recomputed by
`witnessed_at`, preserving evidence outside the affected interval.

## Independent mirrors

A governed witness may sign a mirror observation for an immutable HTTPS URI
containing the checkpoint hash. PostgreSQL commits the observation and its
publication-outbox row atomically. `DESCI_MIN_CHECKPOINT_MIRROR_ORGANIZATIONS`
can require currently valid observations from distinct organizations. See
[Checkpoint Mirrors and Witness Compromises](CHECKPOINT_MIRRORS_AND_COMPROMISES.md).

## What witnessing does not prove

A witness signature does not prove scientific truth. It proves that a separately governed observer signed a particular consistency checkpoint. Witnesses should be operated by administratively and infrastructurally independent organizations, retain checkpoints outside the Mycelix-DeSci deployment, and expose immutable hash-addressed mirror documents that another verifier can fetch and recompute.
